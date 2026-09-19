package thesisfinal;

import java.awt.*;
import java.io.*;
import java.util.ArrayList;
import java.util.Deque;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import static thesisfinal.Constants.*;

import static java.lang.Double.MAX_VALUE;
import static java.lang.Double.NaN;
import static java.lang.Math.*;
import static thesisfinal.Utilities.getDLCModel;

public class Vehicle {

    final static double MARGIN = 1; // margin from segment end in meter
    final static double TIME_STEP = Constants.TIME_STEP; // second
    final static double SAFE_TIME_GAP = 0.0; // second
    final static double LAMBDA = 0.78; // lane changing gap acceptance parameter
    private final static int TIME_WINDOW = 4;
    private final static int GAP_WINDOW = 3;
    private final static double THRESHOLD_DISTANCE = Constants.THRESHOLD_DISTANCE; // default gap in meter between two standstill vehicles
    private final static double REACTION_TIME = Constants.TIME_STEP; // second
    private final static double ALPHA = 15; // sensitivity co-efficient for GHR/General Motors DLC model
    private final static double M = 1; // speed exponent of GHR/General Motors DLC model
    private final static double L = 2; // speed exponent of GHR/General Motors DLC model
    private final int vehicleId;
    private final int startTime;
    private int type;
    private double length;
    private double width;
    private int numberOfStrips;
    private double speed;
    private double acceleration;
    private int stripIndex;
    private double distanceInSegment;   // tail of the vehicle is at distance in segment; so head = tail + length
    private double maximumSpeedCapable; // vehicle's max speed capability
    private double currentMaxSpeed; // max speed limit of the road/topology
    private double maxAcceleration;
    private double maxBraking;  // negative(must)
    private Color color;
    private Point2D[] vehicleCorners;
    private int demandIndex;
    private int pathIndex;
    private int linkIndexOnPath;
    private Point2D[] segmentCorners;
    private Point2D segmentStartPoint;
    private Point2D segmentEndPoint;
    private boolean isInIntersection;
    private boolean reverseLink;
    private boolean reverseSegment;
    private boolean passedSensor;
    private boolean toRemove;
    private Link link;
    private int segmentIndex;
    private Node node;
    private int intersectionStripIndex;
    private double distanceInIntersection;
    private Vehicle leader;
    private int endTime;
    private int segmentEnterTime;
    private int segmentLeaveTime;
    private int waitingTime; // time when the vehicle could not move
    private int waitingTimeInSegment;
    private double distanceTraveled;
    private SIGNAL signalOnLink;
    private Deque<Double> prevSpeeds;   // this is to store #timeWindow previous speeds
    private Deque<Double> prevGaps; // this is to store #gapWindow previous gaps with leader
    private Parameters.DLC_MODEL dlcModel;
    private boolean hasCollided;
    private int collisionTime;
    private int collisionPenalty;

    /**
     * when this variable is true vehicles cannot overlap in node
     * but if this variable is false the vehicle can make a jump to avoid deadlock in intersection
     */
    private boolean noForceMove;
    /**
     * this variable determines for how many seconds a variable is stuck in an intersection
     */
    private int stuckInIntersection;
    private double fuelConsumption;
    private double penaltyForCollision;

    private VehicleStats vehicleStats;

    // Temporal desync attack: position history buffer
    private final List<double[]> positionHistory;
    private int lastRecordedStep;

    private Vehicle() {
        vehicleId = -5;
        startTime = -1;
        positionHistory = null;
        lastRecordedStep = -1;
    }

    public Vehicle(int vehicleId, int startTime, int type, Color color, int demandIndex, int pathIndex, int linkIndexOnPath, double segStartX, double segStartY, double segEndX, double segEndY, boolean reverseLink, boolean reverseSegment, Link link, int segmentIndex, int stripIndex) {
        
        this.vehicleId = vehicleId;
        this.startTime = startTime;
        this.segmentEnterTime = startTime;
        this.distanceTraveled = 0;
        this.type = type;

        this.length = Utilities.getCarLength(type);
        this.width = Utilities.getCarWidth(type);
        this.maximumSpeedCapable = Utilities.getCarMaxSpeed(type);
        this.currentMaxSpeed = min(Parameters.maximumSpeed, maximumSpeedCapable);
        this.numberOfStrips = Utilities.numberOfStrips(type);

        this.speed = 0;//Utilities.getCarMaxSpeed(type);
        this.maxAcceleration = Utilities.getCarAcceleration(type);
        this.acceleration = this.maxAcceleration;
        this.maxBraking = -6; //-1.5 * maxAcceleration;
        this.color = color;
        this.demandIndex = demandIndex;
        this.pathIndex = pathIndex;
        this.linkIndexOnPath = linkIndexOnPath;
        segmentStartPoint = new Point2D(0, 0);
        segmentEndPoint = new Point2D(0, 0);
        this.segmentStartPoint.x = segStartX;
        this.segmentStartPoint.y = segStartY;
        this.segmentEndPoint.x = segEndX;
        this.segmentEndPoint.y = segEndY;

        this.isInIntersection = false;

        this.reverseLink = reverseLink;
        this.reverseSegment = reverseSegment;

        this.passedSensor = false;
        this.toRemove = false;

        this.hasCollided = false;
        this.collisionTime = -1;
        this.collisionPenalty = 0;

        this.noForceMove = false;
        this.stuckInIntersection = 0;

        this.link = link;
        this.segmentIndex = segmentIndex;
        this.stripIndex = stripIndex;
        this.distanceInSegment = 0.1;
        this.vehicleCorners = new Point2D[4];

        this.segmentCorners = new Point2D[4];

        this.prevSpeeds = new LinkedList<>();
        this.prevGaps = new LinkedList<>();
        this.fuelConsumption = 0;
        this.penaltyForCollision = 0;
        this.dlcModel = getDLCModel(type);

        this.vehicleStats = new VehicleStats(this.vehicleId, Parameters.simulationEndTime);

        this.positionHistory = new ArrayList<>(Parameters.POSITION_BUFFER_SIZE);
        this.lastRecordedStep = -1;

        occupyStrips();
        increaseVehicleCountOnSegment();
        getSegment().increaseEnteringVehicleCount();

    }

    public Parameters.DLC_MODEL getDlcModel() {
        return dlcModel;
    }

    public VehicleStats getVehicleStats() {
        return vehicleStats;
    }

    static double getSpeedForBraking(Vehicle leader, Vehicle follower) {
        double Dx = getDx(leader, follower, 1);
        return getSpeedForBraking(leader, follower, Dx);
    }

    static double getSpeedForBraking(Vehicle leader, Vehicle follower, double Dx) {
        double d_n_mx = follower.maxBraking;
        double d_T = d_n_mx * REACTION_TIME;
        double v_n_t = follower.speed;
        double v_n_1_t = leader.speed;
        double d_n_1 = getLeadersPerceivedDeceleration(leader, follower);
        return d_T + sqrt(d_T * d_T - d_n_mx * (2 * Dx - v_n_t * REACTION_TIME - (v_n_1_t * v_n_1_t) / d_n_1));
    }

    private static double getGippsAcceleration(Vehicle leader, Vehicle follower) {
        double Vf = leader.speed; // Vf = leader speed
        double Vb = follower.speed; // preceding car
        double gap = getGap(leader, follower);
        double dt = TIME_STEP;
        double vLen = follower.length;

        double a = follower.maxAcceleration;
        double b = follower.maxBraking;
        double bcap = -3; // according to TA sir
        double s = vLen + THRESHOLD_DISTANCE;
        double Vn = follower.currentMaxSpeed;

        double term1 = Vb + 2.5 * a * dt * (1 - Vb / Vn) * (sqrt(0.025 + Vb / Vn));
        double term2 = (b * b) * (dt * dt) - b * (2 * (gap + vLen - s) - Vb * dt - (Vf * Vf) / bcap);
        double acc;
        if (term2 >= 0) {
            term2 = b * dt + sqrt(term2);
            double Vbnew = min(term1, term2);
            acc = (Vbnew - Vb) / dt;
        } else {
            acc = b;
        }

        return acc;
    }

    private static double getKraussAcceleration(Vehicle leader, Vehicle follower) {
        double Vf = leader.speed; // Vf = leader speed
        double Vb = follower.speed; // preceding car
        double gap = getDx(leader, follower, 1);
        double dt = TIME_STEP;

        double a = follower.maxAcceleration;
        double b = -follower.maxBraking; // according to TA sir's code b is positive
        double s0 = THRESHOLD_DISTANCE; // safe distance = THRESHOLD_DISTANCE
        double Vn = follower.currentMaxSpeed;
//        double tau = 1.0;
        double tau = 0.3;

        double gapDes = Vf * tau;
        double vSafe = Vf + (gap - gapDes) / ((Vb + Vf) / (2 * b) + tau);
        double vDes = min(Vb + a * dt, min(vSafe, Vn));
        double Vbnew = max(0, vDes); // this is the krauss speed
        double acc = (Vbnew - Vb) / dt;
        return acc;
    }

    private static double getGFMAcceleration(Vehicle leader, Vehicle follower) {
        double Vf = leader.speed; // Vf = leader speed
        double Vb = follower.speed; // preceding car
        double gap = getDx(leader, follower, 1);

        double s0 = THRESHOLD_DISTANCE; //safe distance = THRESHOLD_DISTANCE for me
        double Vmax = follower.currentMaxSpeed;
        double T = 1.5;
        double toud = 1.5;
        double toua = 9;
        double Ra = 15;
        double Rd = 80;

        double safeDist = s0 + Vb * T;
        double Vdelx = Vmax * (1 - exp(-(gap - safeDist) / Ra));

        double acc = (Vdelx - Vb) / toua;
        if (Vb > Vf) {
            acc = acc - (Vb - Vf) * exp(-(gap - safeDist) / Rd) / toud;
        }
        return acc;
    }

    static double getIDMAcceleration(Vehicle leader, Vehicle follower) {
        if (follower == null) {
            return 0;
        } else if (leader == null) {
            return follower.maxAcceleration;
        }
        double Vf = leader.speed; // Vf = leader speed
        double Vb = follower.speed; // preceding car
        double gap = getDx(leader, follower, 1);

        double a = 3;
        double b = 4;
        double s0 = THRESHOLD_DISTANCE;
        double Vmax = follower.currentMaxSpeed;
        double T = 1.5;
        double c = 2 * sqrt(a * b);

        double s_star = s0 + max(0, Vb * T + ((Vb * (Vb - Vf)) / c));
        double acc = a * (1 - pow((Vb / Vmax), 4) - pow((s_star / gap), 2));
        acc = max(-6, acc);
        return acc;
    }

    private static double getRVFAcceleration(Vehicle leader, Vehicle follower) {
        double[] Vbs = new double[TIME_WINDOW];
        double[] Vfs = new double[TIME_WINDOW];
        int i = 0;
        for (Double d : follower.prevSpeeds) {
            Vbs[i] = d;
            i++;
        }
        i = 0;
        for (Double d : leader.prevSpeeds) {
            Vfs[i] = d;
            i++;
        }

        double gap = getDx(leader, follower, 1);
        double vLen = follower.length;
        double dt = TIME_STEP;

        double a = 3;
        double b = -3;
        double s0 = THRESHOLD_DISTANCE;
        double Vmax = follower.currentMaxSpeed;
        double cappa = 2.0;
        double V1 = 6.75;
        double V2 = 7.91;
        double C1 = 0.13;
        double C2 = 1.57;
        double lc = follower.length;
        double sc = 140;
        double lambda = 0.5;
        double gamma = 0.2;
        double L = TIME_WINDOW; // sir used 4 but here we used 5

        double s = gap + vLen;
        double Vdelx = V1 + V2 * tanh(toRadians(C1 * gap - C2));
        double acc = cappa * (Vdelx - follower.speed); // Vbs[0] is the previous speed so current speed is use here
        double VfAvg = 0;

        for (i = 0; i < TIME_WINDOW; i++) {
            VfAvg += Vfs[i] - Vbs[i];
        }

        VfAvg /= L;
        acc = acc + lambda * (leader.speed - follower.speed) + gamma * (leader.speed - follower.speed - VfAvg);
        double Vnew = max(0, follower.speed + acc * dt);
        acc = (Vnew - follower.speed) / dt;
        return acc;
    }

    private static double getVFIACAcceleration(Vehicle leader, Vehicle follower) {
        double Vb = follower.speed;
        double[] Vfs = new double[TIME_WINDOW];
        int i = 0;
        for (Double d : leader.prevSpeeds) {
            Vfs[i] = d;
            i++;
        }
        double gap = getDx(leader, follower, 1);
        double vLen = follower.length;
        double dt = TIME_STEP;

        double a = 3;
        double b = -3;
        double s0 = THRESHOLD_DISTANCE;
        double Vmax = follower.currentMaxSpeed;
        double cappa = 0.41;
        double V1 = 6.75;
        double V2 = 7.91;
        double C1 = 0.13;
        double C2 = 1.57;
        double lc = follower.length;
        double sc = 140;
        double lambda = 0.5;
        double gamma = 0.03;
        int m = TIME_WINDOW;

        double s = gap + vLen;
        double Vdelx = V1 + V2 * tanh(toRadians(C1 * gap - C2));
        double acc = cappa * (Vdelx - Vb);
        double VfAvg = 0;

        for (double d : Vfs) {
            VfAvg += d;
        }
        VfAvg /= m;
        acc += lambda * (leader.speed - Vb) + gamma * (leader.speed - VfAvg);
        double Vnew = max(0, Vb + acc * dt);
        acc = (Vnew - Vb) / dt;
        return acc;
    }

    private static double getOVCMAcceleration(Vehicle leader, Vehicle follower) {
        double Vf = leader.speed; // Vf = leader speed
        double Vb = follower.speed; // preceding car
        double gap = getDx(leader, follower, 1);
        double[] gaps = new double[follower.prevGaps.size()];
        int i = 0;
        for (Double d : follower.prevGaps) {
            gaps[i] = d;
            i++;
        }
        double dt = TIME_STEP;

        double Vmax = follower.currentMaxSpeed;
        double a = 2.3;
        double lc = follower.length;
        double hc = 4;
        double lambda = 0.1;
        double memTimeStep = 2; //TODO dt = 1 then what???
        int step = (int) (memTimeStep / dt);
        double gamma = 0.1;


        double Vdelx = Vmax / 2 * (tanh(toRadians(gap + lc - hc)) + tanh(toRadians(hc)));
        double Vdelx2 = Vdelx;
        if (gaps.length > step) {
            Vdelx2 = Vmax / 2 * (tanh(toRadians(gaps[step - 1] + lc - hc)) + tanh(toRadians(hc)));
        }
        double acc = a * (Vdelx - Vb) + lambda * (Vf - Vb) + gamma * (Vdelx - Vdelx2);

        return acc;
    }

    private static double getKFTMAccSingleLeader(Vehicle leader, Vehicle follower, double multfactor, int leaderNo) {
        double ALPHA = Parameters.ALPHA; //0.92, 0.93, ..., 1.09, 1.10
        double BETA = Parameters.BETA; // 0.1, 0.2, 0.3, ..., 1.9, 2.0
        double ETA = Parameters.ETA; // 0, 1, 2, ..., 9, 10

        double Vf = leader.speed; // Vf = leader speed
        double Vb = follower.speed; // preceding car
        double gap = getDx(leader, follower, leaderNo);
        double vLen = follower.length;
        double dt = TIME_STEP;

        double a = 3;
        double b = -4;
        double bcap = -4;
        double s = vLen + 2;
        double s0 = THRESHOLD_DISTANCE;
        double Vn = follower.currentMaxSpeed;
        double c = 2 * sqrt(a * abs(b));
        double T = 0.9;

        double s_star = (s0 + ETA + max(0, Vb * T + (Vb * (Vb - Vf)) / c)) * BETA;

        double Va_nm = Vb + a * dt * (1 - pow(Vb / Vn, 4) - pow(((multfactor * s_star / gap) * (Vb / Vn)), 2));

        double part1 = (b * b) * (dt * dt);
        double part2 = 2 * (gap + vLen - s) - Vb * dt - ((Vf * Vf) / bcap);
        double term2 = part1 - b * part2;

        double Vb_nm;
        if (term2 >= 0) {
            Vb_nm = b * dt + sqrt(term2);
        } else {
            Vb_nm = b * dt + Vb;
        }
        Vb_nm = ALPHA * Vb_nm;

        double Vbnew = max(0, min(Va_nm, Vb_nm));

        return (Vbnew - Vb) / dt; // acceleration
    }

    private static double getKFTMAcceleration(Vehicle leader, Vehicle follower) {
        double lookAheadDist = 300;

        Vehicle l = leader;
        double minAcc = getKFTMAccSingleLeader(leader, follower, 1, 1);

        for (int i = 1; i < 3; i++) {
            if (l != null && l.distanceInSegment - follower.distanceInSegment <= lookAheadDist) {
                double tempAcc = getKFTMAccSingleLeader(l, follower, i + 1, i + 1);
                minAcc = min(tempAcc, minAcc);

                l = l.getProbableLeader();
            } else {
                break;
            }
        }
        return minAcc;
    }

    //TODO
    private static double getHDMAcceleration(Vehicle leader, Vehicle follower) {
        int numLeaderToConsider = 3;

        // follower's leader, then his leader, his leader, ...
        ArrayList<Vehicle> leaders = new ArrayList<>();

        leaders.add(leader);
        int count = 1;
        Vehicle l = leader;
        while (count <= numLeaderToConsider) {
            l = l.getALeaderAsNecessary();
            if (l == null) {
                break;
            }
            leaders.add(l);
            count++;
        }

        int numOfActualLeaders = leaders.size();

        /*
            in index 0 -> my vehicle(follower)
               index 1 -> leader of 0
               index 2 -> leader of 1
               .....
            for both speeds and gap (different from Tanveer sir)
         */
        double[] speeds = new double[numOfActualLeaders + 1];
        double[] gaps = new double[numOfActualLeaders + 1];

        speeds[0] = follower.speed;
        for (int i = 1; i <= numOfActualLeaders; i++) {
            speeds[i] = leaders.get(i - 1).speed;
        }

        gaps[0] = getDx(leader, follower, 1);
        for (int i = 1; i < numOfActualLeaders; i++) {
            Vehicle l1 = leaders.get(i);
            Vehicle f1 = leaders.get(i - 1);
            gaps[i] = getDx(l1, f1, i + 1); //gap between f1 and l1
        }

        Vehicle farthestCar = leaders.get(numOfActualLeaders - 1);
        Vehicle leaderOfTheFarthestCar = farthestCar.getALeaderAsNecessary();
        if (leaderOfTheFarthestCar == null) {
            gaps[numOfActualLeaders] = 1000;
        } else {
            gaps[numOfActualLeaders] = getDx(leaderOfTheFarthestCar, farthestCar, numOfActualLeaders);
        }

        /*for (int i = 0; i < numOfActualLeaders; i++) {
            int index = numOfActualLeaders - 1 - i;
            speeds[i] = leaders.get(index).speed;
        }
        speeds[numOfActualLeaders] = follower.speed;

        Vehicle farthestCar = leaders.get(numOfActualLeaders - 1);
        Vehicle leaderOfTheFarthestCar = farthestCar.getProbableLeader();
        if (leaderOfTheFarthestCar == null) {
            gaps[0] = 10000;
        } else {
            gaps[0] = getDx(leaderOfTheFarthestCar, farthestCar);
        }

        for (int i = 1; i < numOfActualLeaders; i++) {
            gaps[i] = getDx(leaders.get(i + 1), leaders.get(i));
        }

        gaps[numOfActualLeaders] = getDx(leader, follower);*/

        double acc = getHDMAccHelper(follower, numOfActualLeaders,
                speeds, gaps, TIME_STEP);

        return acc;
    }

    static double getHDMAccHelper(Vehicle follower, int numLeaders,
                                  double[] speeds, double[] gaps, double dt) {
        double acc = follower.acceleration;

        double a = follower.maxAcceleration;
        double b = follower.maxBraking;
        double s0 = THRESHOLD_DISTANCE;

        double vMax = follower.currentMaxSpeed;
        double T = 1.5;
        double c = 2 * sqrt(a * abs(b));

        double fs = follower.speed;

        double[] accInteraction = new double[numLeaders];

        double vNext = fs + acc * dt;
        vNext = max(0, min(vNext, vMax));

        for (int i = 1; i <= numLeaders; i++) {
            double delV = fs - speeds[i];
            double s_star = s0 + max(0, vNext * T + vNext * delV / c);

            double gap = gaps[0] + dt * (fs - speeds[1]);
            for (int j = 1; j < i; j++) {
                gap += gaps[j];
            }

            accInteraction[i - 1] = (s_star / gap) * (s_star / gap);
        }

        double sum = 0;
        for (double d : accInteraction)
            sum += d;

        double resAcc = a * (1 - pow(vNext / vMax, 4)) - a * sum;
        resAcc = max(follower.maxBraking, resAcc);

        return resAcc;
    }


    static double getSBMAcceleration(Vehicle leader, Vehicle follower) {
        double Vf = leader.speed; // Vf = leader speed
        double Vb = follower.speed; // preceding car
        double gap = getDx(leader, follower, 1);
        double vLen = follower.length;
        double deltaT = TIME_STEP;

        double vMax = follower.currentMaxSpeed;
        double gamma = 2.0;
        double accMaxN = 3.0;
        double dJam = 2.0;

        double dRep = (Vb / (2.5 + 0.1 * Vb)) * vLen + dJam + (0.0 + Parameters.random.nextGaussian() * 0.05);
        double dPar = gamma * dRep;

        double phi;
        if ((Vb - Vf) > ((gap + vLen) / (2 * Vb))) {
            phi = 1;
        } else {
            phi = 2.4;
        }

        double Vb_new = 0;
        double acc = 0;
        if ((gap + vLen) < dRep && Vf != 0) {
            Vb_new = Vb + (gap + vLen - dRep) / (phi * deltaT) + (0.0 + Parameters.random.nextGaussian() * 0.05);
            acc = (Vb_new - Vb) / deltaT;
        } else if ((gap + vLen) > dRep && (gap + vLen) < dPar) {
            double val = (0.1 + Parameters.random.nextGaussian() * (Vf / vMax));
            Vb_new = Vf * val;
            acc = (Vb_new - Vb) / deltaT;
        } else if ((gap + vLen) > dPar) {
            double vFeasible = Vb + accMaxN * deltaT;
            Vb_new = min(vMax, vFeasible);
            Vb_new = min(Vb_new, (Vf * (gap + vLen) / vLen));
            acc = (Vb_new - Vb) / deltaT;
        } else {
            acc = 2;
        }

        acc = max(-6, acc);
        return acc;
    }

    /**
     * @param leader   vehicle in front (1, 2, 3, 4 th vehicle in front)
     * @param follower current vehicle (0 th vehicle)
     * @param leaderNo 1 means immediate leader; 2 means leader of leader; and so on ...
     * @return gap between leader and follower
     */
    static double getDx(Vehicle leader, Vehicle follower, int leaderNo) {
        if (Parameters.ERROR_MODE) {
            if (Parameters.FT_METHOD == 1) {
                return getDxWithError(leader, follower, leaderNo, Parameters.NO_OF_READINGS, 0);
            } else if (Parameters.FT_METHOD == 2) {
                return getDxWithError(leader, follower, leaderNo, 1, Parameters.M_FACTOR);
            } else if (Parameters.FT_METHOD == 3) {
                return getDxWithError(leader, follower, leaderNo, Parameters.NO_OF_READINGS, Parameters.M_FACTOR);
            } else if (Parameters.FT_METHOD == 4) {
                return getDxTemporalDesync(leader, follower, leaderNo);
            } else {
                assert false;
                return 0; // should never come here. FT_METHOD should be 1, 2, 3, or 4
            }
        } else {
            double g = getGap(leader, follower);
            if (Parameters.FT_METHOD == 4 && leaderNo == 1) {
                Statistics.addGapError(leader, follower, 0, g, g, Parameters.simulationStep);
            }
            return g;
        }
    }

    static double getGap(Vehicle leader, Vehicle follower) {
        double x_n_1 = leader.distanceInSegment;
        double x_n = follower.distanceInSegment + follower.length;
//        double s_n_1 = leader.length;
        // TODO look here
        return x_n_1 - THRESHOLD_DISTANCE - x_n;
    }

    static double getObjectGap(Object leader, Vehicle follower) {
        double x_n_1 = leader.getDistanceInSegment();
        double x_n = follower.distanceInSegment + follower.length;//       
        return x_n_1 - THRESHOLD_DISTANCE - x_n;
    }

    static double getDxWithError(Vehicle leader, Vehicle follower, int leaderNo, int noOfReadings, double mFactor) {
        if (leaderNo == 1) {
            // immediate leader so Radar error
            return getDxWithRadarError(leader, follower, Parameters.NO_OF_READINGS, Parameters.M_FACTOR);
        } else {
            // not immediate leader, so we need to get position through GPS
            // and get gap through inter vehicle communication
            return getDxWithPositionError(leader, follower, Parameters.NO_OF_READINGS, Parameters.M_FACTOR);
        }
    }

    static double getDxWithRadarError(Vehicle leader, Vehicle follower, int noOfReadings, double mFactor) {
        //TODO bound parameterization
        double sigmaRadar = 2;
        double boundRadar = 0.38;
        // we want the gap to be smallest; error can be negative; so we need the smallest value
        double maxError = Double.MAX_VALUE;
        for (int i = 0; i < noOfReadings; i++) {
            double temp = Utilities.truncatedGaussian(sigmaRadar, -boundRadar, boundRadar);
            if (temp < maxError) {
                maxError = temp;
            }
        }
        return getGap(leader, follower) + maxError - boundRadar * mFactor;
    }

    static double getDxWithPositionError(Vehicle leader, Vehicle follower, int noOfReadings, double mFactor) {
        //TODO bound parameterization
        double sigmaPos = 10;
        double boundPos = 10;
        double maxLeaderPos = Double.MAX_VALUE;
        double maxFollowerPos = Double.MIN_VALUE;
        for (int i = 0; i < noOfReadings; i++) {
            double temp = Utilities.truncatedGaussian(sigmaPos, -boundPos, boundPos);
            if (temp < maxLeaderPos) {
                maxLeaderPos = temp;
            }
            temp = Utilities.truncatedGaussian(sigmaPos, -boundPos, boundPos);
            if (maxFollowerPos < temp) {
                maxFollowerPos = temp;
            }
        }
        double x_n_1 = leader.distanceInSegment + maxLeaderPos - boundPos * mFactor;
        double x_n = follower.distanceInSegment + maxFollowerPos + follower.length + boundPos * mFactor;
        //double s_n_1 = THRESHOLD_DISTANCE;
        return x_n_1 - x_n;
    }

    private static double getLeadersPerceivedDeceleration(Vehicle leader, Vehicle follower) {
        return (leader.maxBraking + follower.maxBraking) / 2.0;
    }

    /**
     * @return this method returns the minimum distance between leader and follower to achieve desired speed of follower
     */
    private static double getDistanceForDesiredSpeed(Vehicle leader, Vehicle follower) {
        double Vd = follower.currentMaxSpeed;
        double T = TIME_STEP;
        double d_n_mx = follower.maxBraking;
        double v_n = follower.speed;
        double v_n_1 = leader.speed;
        double d_n_1 = getLeadersPerceivedDeceleration(leader, follower);
        return Vd * T + (v_n * T + v_n_1 * v_n_1 / d_n_1 - Vd * Vd / d_n_mx) / 2.0;
    }

    private static double getObjectDistanceForDesiredSpeed(Vehicle follower) {
        double Vd = follower.currentMaxSpeed;
        double T = TIME_STEP;
        double d_n_mx = follower.maxBraking;
        double v_n = follower.speed;        
        return Vd * T + (v_n * T - Vd * Vd / d_n_mx) / 2.0;
    }

    static double getAccelerationGHRModel(Vehicle leader, Vehicle follower) {
        double v = follower.speed;
        double delV = leader.speed - follower.speed;
        double delX = getGap(leader, follower);
        double ans = ALPHA * Math.pow(v, M) * delV / Math.pow(delX, L);
        return ans;
    }

    private void occupyStrips() {
        Segment segment = link.getSegment(segmentIndex);
        for (int i = 0; i < numberOfStrips; i++) {
            segment.getStrip(stripIndex + i).addVehicle(this);
        }
    }

    private void increaseVehicleCountOnSegment() {
        Segment segment = link.getSegment(segmentIndex);
        if (isReverseSegment()) {
            segment.increaseReverseVehicleCount();
        } else {
            segment.increaseForwardVehicleCount();
        }
    }

    void decreaseVehicleCountOnSegment() {
        Segment segment = link.getSegment(segmentIndex);
        if (isReverseSegment()) {
            segment.decreaseReverseVehicleCount();
        } else {
            segment.decreaseForwardVehicleCount();
        }
    }

    void freeStrips() {
        Segment segment = link.getSegment(segmentIndex);
        for (int i = stripIndex; i < stripIndex + numberOfStrips; i++) {
            segment.getStrip(i).delVehicle(this);
        }

    }

    private void storePrevSpeeds() {
        prevSpeeds.addFirst(speed);
        if (prevSpeeds.size() > TIME_WINDOW) {
            prevSpeeds.removeLast();
        }
    }

    /*private double getNewSpeedGippsModel() {
        double v_a = speed + maxAcceleration * TIME_STEP;
        double v_b;
        Vehicle leader = getProbableLeader();
        if (leader == null) {
            v_b = v_a;
        } else {
            acceleration = getGippsAcceleration(leader, this);
            v_b = speed + acceleration * TIME_STEP;
        }

        return Utilities.precision2(max(0.0, min(v_a, v_b)));
    }*/

    private void storePrevGaps() {
        Vehicle leader = getProbableLeader();
        double gap = 100000;
        if (leader != null) {
            gap = getDx(leader, this, 1);
        }
        prevGaps.addFirst(gap);
        if (prevGaps.size() > GAP_WINDOW) {
            prevGaps.removeLast();
        }

//        for (Double d : prevGaps) {
//            System.out.print(d + " ");
//        }
//        System.out.println();
    }

    /**
     * @param numLead number of leaders to be considered
     * @return gaps between me and the numLead leaders
     */
    private double[] getNLeadersGaps(int numLead) {
        int c = 0; // # of actual leaders
        Vehicle leader = getProbableLeader();
        while (leader != null) {
            c++;
            leader = leader.getProbableLeader();
        }
        int N = Math.min(c, numLead); // actual leaders to be considered
        double[] gaps = new double[N];
        Vehicle follower = this;
        leader = follower.getProbableLeader();
        for (int i = 0; i < N; i++) {
            gaps[i] = getGap(leader, follower);
            follower = leader;
            leader = follower.getProbableLeader();
        }
        return gaps;
    }

    @SuppressWarnings("Duplicates")
    private boolean moveToHigherIndexLane() {
        Segment segment = link.getSegment(segmentIndex);

        int limit = isReverseSegment() ? segment.lastVehicleStripIndex : segment.middleLowStripIndex;

        int storedStripIndex = stripIndex;

        Vehicle leader = this.getProbableLeader();
        Vehicle follower = this.getProbableFollower();

        for (int i = stripIndex; i + numberOfStrips <= limit; i++) {
            changeStrip(i, i + numberOfStrips, 1);
            if (segment.getStrip(i + numberOfStrips).hasGapForStripChange(this, leader, follower)) {

                if(isObjectInProximity()) return true; // if object present, having gap is enough, no need to check for other conditions
                else if (isMoveForwardPossible() && !isSlowerVehicleInProximity()) {
                    moveForwardInSegment();
                    return true;
                }
            } else {
                break;
            }
        }

        freeStrips();
        stripIndex = storedStripIndex;
        occupyStrips();
        return false;
    }

    @SuppressWarnings("Duplicates")
    private boolean moveToLowerIndexLane() {
        Segment segment = link.getSegment(segmentIndex);

        int limit = isReverseSegment() ? segment.middleHighStripIndex : 1;

        int storedStripIndex = stripIndex;

        Vehicle leader = this.getProbableLeader();
        Vehicle follower = this.getProbableFollower();

        for (int i = stripIndex; i > limit; i--) {
            changeStrip(i + numberOfStrips - 1, i - 1, -1);
            if (segment.getStrip(i - 1).hasGapForStripChange(this, leader, follower)) {

                if(isObjectInProximity()) return true;  // if object present, having gap is enough, no need to check for other conditions
                else if (isMoveForwardPossible() && !isSlowerVehicleInProximity()) {
                    moveForwardInSegment();
                    return true;
                }
            } else {
                break;
            }
        }

        freeStrips();
        stripIndex = storedStripIndex;
        occupyStrips();
        return false;
    }

    /**
     * helper function to simplify changing strip
     *
     * @param removeFrom vehicle is removed from this strip
     * @param addTo      vehicle is added to this strip
     * @param deltaIndex change of index (+/-1)
     */
    private void changeStrip(int removeFrom, int addTo, int deltaIndex) {
        Segment segment = link.getSegment(segmentIndex);

        segment.getStrip(removeFrom).delVehicle(this);
        segment.getStrip(addTo).addVehicle(this);
        stripIndex = stripIndex + deltaIndex;
    }

    // Butcher's Method
    private double getNewDistanceInSegment() {
        
        double vNew = speed;
        double vOld = prevSpeeds.getFirst();
        double delV = vNew - vOld;

        double k1 = vOld;
        double k3 = vOld + 0.25 * delV;
        double k4 = vOld + 0.50 * delV;
        double k5 = vOld + 0.75 * delV;
        double k6 = vNew;

        double newDistanceInSegment = distanceInSegment + (1.0 / 90) * (7 * k1 + 32 * k3 + 12 * k4 + 32 * k5 + 7 * k6 ) * TIME_STEP;
        return newDistanceInSegment;
//        return distanceInSegment + speed * TIME_STEP;
    }

    private boolean isBrakeRequiredForLeader() {
        assert getProbableLeader() == null;
        double s = (currentMaxSpeed * currentMaxSpeed) / (2 * -maxBraking); // v^2 = u^2 - 2as; we get s from this eqn
        double distFromSegEnd = getDistanceFromSegmentEnd();
        return s <= distFromSegEnd;
    }

    private double getSpeedVehicleIfLeader() {
        assert getProbableLeader() == null;
        double v;
        if (isBrakeRequiredForLeader()) {
            v = sqrt(speed * speed + 2 * maxBraking * getDistanceFromSegmentEnd());
        } else {
            v = speed + maxAcceleration * TIME_STEP;
        }
        acceleration = (v - speed) / TIME_STEP;
        return v;
    }

    /**
     * optimize this code so that every time we need not generate a new dummy
     *
     * @return a virtual stopped car at segment end
     */
    private Vehicle createDummyVehicleAtLinkEnd(SIGNAL signal) {
        Vehicle v = new Vehicle();
        if (signal == SIGNAL.GREEN) {
            v.speed = 5;
        } else if (signal == SIGNAL.RED) {
            v.speed = 0;
        }
        v.acceleration = 0;
        v.maxAcceleration = maxAcceleration;
        v.maxBraking = maxBraking;
        v.type = type;
        v.length = 0.1;
        v.maximumSpeedCapable = maximumSpeedCapable;
        v.currentMaxSpeed = currentMaxSpeed;
        v.distanceInSegment = getLink().getSegment(getSegmentIndex()).getLength();
        v.prevSpeeds = new LinkedList<>();
        v.prevGaps = new LinkedList<>();
        for (int i = 0; i < TIME_WINDOW; i++) {
            v.prevSpeeds.addFirst(0.0);
        }
        for (int i = 0; i < GAP_WINDOW; i++) {
            v.prevGaps.addFirst(MAX_VALUE);
        }
        v.reverseLink = reverseLink;
        v.link = link;
        v.segmentIndex = segmentIndex;
        return v;
    }

    private Vehicle createDummyVehicleAtInfinity() {
        Vehicle v = new Vehicle();
        v.speed = currentMaxSpeed;
        v.acceleration = maxAcceleration;
        v.maxAcceleration = maxAcceleration;
        v.maxBraking = maxBraking;
        v.type = type;
        v.length = 0.1;
        v.maximumSpeedCapable = maximumSpeedCapable;
        v.currentMaxSpeed = currentMaxSpeed;
        v.distanceInSegment = 100000000;
        v.prevSpeeds = new LinkedList<>();
        v.prevGaps = new LinkedList<>();
        for (int i = 0; i < TIME_WINDOW; i++) {
            v.prevSpeeds.addFirst(currentMaxSpeed);
        }
        for (int i = 0; i < GAP_WINDOW; i++) {
            v.prevGaps.addFirst(MAX_VALUE);
        }
        v.reverseLink = reverseLink;
        v.link = link;
        v.segmentIndex = segmentIndex;
        return v;
    }

    Vehicle createDummyVehicleAtPedestrianPositionForMyModel(Pedestrian pedestrian) {
        Vehicle v = new Vehicle();
        v.speed = 0;
        v.acceleration = 0;
        v.maxAcceleration = maxAcceleration;
        v.maxBraking = maxBraking;
        v.type = 12;
        v.length = 0.4;
        v.maximumSpeedCapable = pedestrian.getSpeed();
        v.currentMaxSpeed = pedestrian.getSpeed();
        v.prevSpeeds = new LinkedList<>();
        v.prevGaps = new LinkedList<>();
        for (int i = 0; i < TIME_WINDOW; i++) {
            v.prevSpeeds.addFirst(0.0);
        }
        for (int i = 0; i < GAP_WINDOW; i++) {
            v.prevGaps.addFirst(MAX_VALUE);
        }
        Segment segment = pedestrian.getSegment();
        v.stripIndex = pedestrian.getStrip().getStripIndex();
        v.numberOfStrips = Utilities.numberOfStrips(v.type);
        v.reverseLink = reverseLink;
        v.reverseSegment = reverseSegment;
        v.link = link;
        v.segmentIndex = segmentIndex;
        assert v.segmentIndex == segment.getIndex();
        v.distanceInSegment = pedestrian.getDistanceInSegment();
        return v;
    }
    /**
     * this gets the leader
     * if this is the first car and in the last segment and has a red light then
     * it will create and return a dummy vehicle as leader
     * other wise returns the leader from probable leader
     *
     * @return
     */
    private Vehicle getALeaderAsNecessary() {
        Vehicle leader = getProbableLeader();
        Segment lastSegmentForThisInCurrentLink = this.isReverseLink() ? link.getFirstSegment() : link.getLastSegment();

        if (leader == null) {
            if (lastSegmentForThisInCurrentLink == getSegment()) {
                leader = createDummyVehicleAtLinkEnd(signalOnLink);
            } else {
                leader = createDummyVehicleAtInfinity();
            }
        }
//        this.leader = leader;
        return leader;
    }

    private Vehicle getALeaderAsNecessaryForMyModel(int sideStripsToConsider) {
            Vehicle leader = getMyProbableLeader(sideStripsToConsider);
            Segment lastSegmentForThisInCurrentLink = this.isReverseLink() ? link.getFirstSegment() : link.getLastSegment();

            if (leader == null) {
                if (lastSegmentForThisInCurrentLink == getSegment()) {
                    leader = createDummyVehicleAtLinkEnd(signalOnLink);
                } else {
                    leader = createDummyVehicleAtInfinity();
                }
            }
            return leader;
    }

    public void printStat() {
        if (this.vehicleId == 20) {
            System.out.printf("T: %d  .  S: %.2f  .   A: %.2f\n", Parameters.simulationStep, this.speed, this.acceleration);
        }
    }

    public void addStats() {
        vehicleStats.addSpeed(speed, Parameters.simulationStep);
        vehicleStats.addPointOnTrajectory(vehicleCorners[0], Parameters.simulationStep);
    }

    public void recordPosition() {
        double[] snapshot = new double[]{Parameters.simulationStep, distanceInSegment, speed, stripIndex};
        if (positionHistory.size() >= Parameters.POSITION_BUFFER_SIZE) {
            positionHistory.remove(0);
        }
        positionHistory.add(snapshot);
        lastRecordedStep = Parameters.simulationStep;
    }

    private double getHistoricalDistanceInSegment(int stepsAgo) {
        if (positionHistory == null || positionHistory.isEmpty()) return distanceInSegment;
        int targetStep = Parameters.simulationStep - stepsAgo;
        for (int i = positionHistory.size() - 1; i >= 0; i--) {
            double[] entry = positionHistory.get(i);
            if (entry[0] <= targetStep) {
                return entry[1];
            }
        }
        return positionHistory.get(0)[1];
    }

    private static double getDxWithDelay(Vehicle leader, Vehicle follower, int delaySteps) {
        double x_n_1 = leader.getHistoricalDistanceInSegment(delaySteps);
        double x_n = follower.distanceInSegment + follower.length;
        return x_n_1 - THRESHOLD_DISTANCE - x_n;
    }

    private static double getDxTemporalDesync(Vehicle leader, Vehicle follower, int leaderNo) {
        if (leaderNo != 1) return getGap(leader, follower);

        int step = Parameters.simulationStep;

        // Check if attack is active at this time step
        boolean attackActive = true;
        if (step < Parameters.ATTACK_START_TIME) attackActive = false;
        if (Parameters.ATTACK_END_TIME > 0 && step > Parameters.ATTACK_END_TIME) attackActive = false;
        if (Parameters.ATTACK_STRIP_MOD > 0 && (follower.stripIndex % Parameters.ATTACK_STRIP_MOD != 0)) attackActive = false;
        if (Parameters.ATTACK_STRIP_MOD == 0) attackActive = false;

        if (!attackActive) return getGap(leader, follower);

        double gap;
        switch (Parameters.ATTACK_SCENARIO) {
            case 0: // Persistent delay
                int delaySteps = (int) Math.round(Parameters.DELAY_MEAN / Constants.TIME_STEP);
                gap = getDxWithDelay(leader, follower, Math.max(1, delaySteps));
                break;
            case 1: // Intermittent faults
                if (Parameters.random.nextDouble() < Parameters.DELAY_PROBABILITY) {
                    int intDelay = (int) Math.round(Parameters.DELAY_MEAN / Constants.TIME_STEP);
                    gap = getDxWithDelay(leader, follower, Math.max(1, intDelay));
                } else {
                    gap = getGap(leader, follower);
                }
                break;
            case 2: // Single-shot
                if (step == Parameters.SINGLE_SHOT_TIME) {
                    int ssDelay = (int) Math.round(Parameters.SINGLE_SHOT_DELAY / Constants.TIME_STEP);
                    gap = getDxWithDelay(leader, follower, Math.max(1, ssDelay));
                } else {
                    gap = getGap(leader, follower);
                }
                break;
            case 3: // Jitter-only
                double jitteredPos = leader.distanceInSegment
                        + Utilities.truncatedGaussian(Parameters.JITTER_MAGNITUDE, -5, 5);
                double x_n_1 = jitteredPos;
                double x_n = follower.distanceInSegment + follower.length;
                gap = x_n_1 - THRESHOLD_DISTANCE - x_n;
                break;
            case 4: // Packet drop (use oldest available or 5s stale)
                if (Parameters.random.nextDouble() < Parameters.PACKET_LOSS_RATE) {
                    int maxStale = (int) Math.round(5.0 / Constants.TIME_STEP);
                    gap = getDxWithDelay(leader, follower, maxStale);
                } else {
                    gap = getGap(leader, follower);
                }
                break;
            case 5: // Joint multi-sensor
                double jPos = leader.distanceInSegment
                        + Utilities.truncatedGaussian(Parameters.JITTER_MAGNITUDE, -5, 5);
                double jDist = jPos;
                if (Parameters.random.nextDouble() < Parameters.DELAY_PROBABILITY) {
                    int jDelay = (int) Math.round(Parameters.DELAY_MEAN / Constants.TIME_STEP);
                    jDist = leader.getHistoricalDistanceInSegment(Math.max(1, jDelay));
                }
                double jFollower = follower.distanceInSegment + follower.length;
                gap = jDist - THRESHOLD_DISTANCE - jFollower;
                break;
            default:
                gap = getGap(leader, follower);
        }

        // Record the gap error for statistics
        double trueGap = getGap(leader, follower);
        Statistics.addGapError(leader, follower, gap - trueGap, gap, trueGap, Parameters.simulationStep);

        return gap;
    }

    private double getSpeedForAcceleration() {
        double v_n_t = speed;
        double a_n_mx = maxAcceleration;
        double v_n_desired = currentMaxSpeed;
        return v_n_t + 2.5 * a_n_mx * REACTION_TIME * (1 - v_n_t / v_n_desired)
                * sqrt(0.025 + v_n_t / v_n_desired);
    }

    private double getLocalDensity() {
        int startStripLocal = getStartStripForMyModel(numberOfStrips);
        int endStripLocal = getEndStripForMyModel(numberOfStrips);
        double startingDistance = this.distanceInSegment;
        double endingDistance = this.distanceInSegment + this.length * 2.0;

        HashSet<Integer> neighborVehicles = new HashSet<>();
        HashSet<Integer> neighborPedestrians = new HashSet<>();


        for (int i = startStripLocal; i <= endStripLocal; i++) {
            Strip s = this.getSegment().getStrip(i);
            neighborVehicles.addAll(s.getVehiclesInRange(startingDistance, endingDistance));
            neighborPedestrians.addAll(s.getPedestriansInRange(startingDistance, endingDistance));
        }
        double totalNeighbors = neighborVehicles.size() + neighborPedestrians.size();
        double density = totalNeighbors / 9;
        return density;
    }

    private double getMyModelFreeSpeed() {
        return speed + maxAcceleration * TIME_STEP;
    }
    private double getMyModelCongestedSpeed(int sideStripsToConsider) {
        Vehicle closestLeader = getALeaderAsNecessaryForMyModel(sideStripsToConsider);
        double speedClosest = getSpeedForBraking(closestLeader, this);


        ArrayList<Vehicle> leaders = getAllLeadersForMyModel(sideStripsToConsider);

        if (leaders.isEmpty()) {
            return speedClosest;
        }

        ArrayList<Double> speeds = new ArrayList<>();
        ArrayList<Double> weights = new ArrayList<>();
        if (Parameters.CONSIDER_MINIMUM) {
            double minSpeed = speedClosest;
            for (Vehicle leader : leaders) {
                double speedWRTLeader = getSpeedForBraking(leader, this);
                minSpeed = Double.min(minSpeed, speedWRTLeader);
            }
            return minSpeed;
        } else {
            double totalWeight = 0;
            for (Vehicle leader : leaders) {
                double speedWRTLeader = getSpeedForBraking(leader, this);
                double weightWRTLeader = Utilities.getWeight(leader, this, sideStripsToConsider);
                double weightByType = Utilities.getWeightByType(leader);
                double weight = weightWRTLeader * weightByType;

                totalWeight += weight;

                speeds.add(speedWRTLeader);
                weights.add(weight);
            }
            double speed = 0;
            for (int i = 0; i < leaders.size(); i++) {
                speed += weights.get(i) / totalWeight * speeds.get(i);// * Utilities.getWeightByType(leaders.get(i));
            }
            return min(speed, speedClosest);
        }

    }
    private double getNewSpeedMyModel() {
        double v_a = min(currentMaxSpeed, getMyModelFreeSpeed());
//        double v_a = getSpeedForAcceleration();
        double v_b = v_a;

        int sideStripsToConsider = Parameters.SIDE_STRIPS_TO_CONSIDER;

        if (type != PEDESTRIANS_ALONG_THE_ROAD_TYPE) {
            double density = this.getLocalDensity();

            if (density / 10.0 < (1.0 * Parameters.DENSITY_PERCENTAGE / 100.0)) {
                sideStripsToConsider = 0;
            }
        }

        Vehicle leader = getALeaderAsNecessaryForMyModel(sideStripsToConsider);

        if (leader != null) {
            v_b = getMyModelCongestedSpeed(sideStripsToConsider);
        }

        double v = Utilities.precision2(max(0.0, min(v_a, v_b)));
        acceleration = (v - speed) / TIME_STEP;
        return v;
    }

    private double getNewSpeedGippsModel() {
        double v_a = getSpeedForAcceleration();
        double v_b = v_a;

        Vehicle leader = getALeaderAsNecessary();

        if (leader != null) {
            v_b = getSpeedForBraking(leader, this);
        }

        double v = Utilities.precision2(max(0.0, min(v_a, v_b)));
        acceleration = (v - speed) / TIME_STEP;
        return v;
    }

    private double getNewSpeedKraussModel() {
        double v_a = speed + maxAcceleration * TIME_STEP;
        double v_b;

        Vehicle leader = getALeaderAsNecessary();

        if (leader == null) {
            v_b = v_a;
        } else {
            acceleration = getKraussAcceleration(leader, this);
            v_b = speed + acceleration * TIME_STEP;
        }

        return Utilities.precision2(max(0.0, min(v_a, v_b)));
    }

    private double getNewSpeedGFMModel() {
        double v_a = speed + maxAcceleration * TIME_STEP;
        double v_b;

        Vehicle leader = getALeaderAsNecessary();

        if (leader == null) {
            v_b = v_a;
        } else {
            acceleration = getGFMAcceleration(leader, this);
            v_b = speed + acceleration * TIME_STEP;
        }

        return Utilities.precision2(max(0.0, min(v_a, v_b)));
    }

    private double getNewSpeedIDMModel() {
        double v_a = speed + maxAcceleration * TIME_STEP;
        double v_b;

        Vehicle leader = getALeaderAsNecessary();

        if (leader == null) {
            v_b = v_a;
        } else {
            acceleration = getIDMAcceleration(leader, this);
            v_b = speed + acceleration * TIME_STEP;
        }

        double v = Utilities.precision2(max(0.0, min(v_a, v_b)));
        acceleration = (v - speed) / TIME_STEP;
        return v;
    }

    private double getNewSpeedRVFModel() {
        double v_a = speed + maxAcceleration * TIME_STEP;
        double v_b;

        Vehicle leader = getALeaderAsNecessary();

        if (leader == null) {
            v_b = v_a;
        } else {
            acceleration = getRVFAcceleration(leader, this);
            v_b = speed + acceleration * TIME_STEP;
        }

        return Utilities.precision2(max(0.0, min(v_a, v_b)));
    }

    private double getNewSpeedVFIACModel() {
        double v_a = speed + maxAcceleration * TIME_STEP;
        double v_b;

        Vehicle leader = getALeaderAsNecessary();

        if (leader == null) {
            v_b = v_a;
        } else {
            acceleration = getVFIACAcceleration(leader, this);
            v_b = speed + acceleration * TIME_STEP;
        }

        return Utilities.precision2(max(0.0, min(v_a, v_b)));
    }

    private double getNewSpeedOVCMModel() {
        double v_a = speed + maxAcceleration * TIME_STEP;
        double v_b;

        Vehicle leader = getALeaderAsNecessary();

        if (leader == null) {
            v_b = v_a;
        } else {
            acceleration = getOVCMAcceleration(leader, this);
            v_b = speed + acceleration * TIME_STEP;
        }

        return Utilities.precision2(max(0.0, min(v_a, v_b)));
    }

    private double getNewSpeedKFTMModel() {
        double v_a = speed + maxAcceleration * TIME_STEP;
        double v_b;

        Vehicle leader = getALeaderAsNecessary();

        if (leader == null) {
            v_b = v_a;
        } else {
            acceleration = getKFTMAcceleration(leader, this);
            v_b = speed + acceleration * TIME_STEP;
        }

        return Utilities.precision2(max(0.0, min(v_a, v_b)));
    }

    private double getNewSpeedHDMModel() {
        double v_a = speed + maxAcceleration * TIME_STEP;
        double v_b;

        Vehicle leader = getALeaderAsNecessary();

        if (leader == null) {
            v_b = v_a;
        } else {
            acceleration = getHDMAcceleration(leader, this);
            v_b = speed + acceleration * TIME_STEP;
        }

        return Utilities.precision2(max(0.0, min(v_a, v_b)));
    }

    private double getNewSpeedSBMModel() {
        double v_a = speed + maxAcceleration * TIME_STEP;
        double v_b;

        Vehicle leader = getALeaderAsNecessary();

        if (leader == null) {
            v_b = v_a;
        } else {
            acceleration = getSBMAcceleration(leader, this);
            v_b = speed + acceleration * TIME_STEP;
        }

        return Utilities.precision2(max(0.0, min(v_a, v_b)));
    }

    /**
     * @return new speed according to Gipp's model
     */
    double getNewSpeed() {
        switch (Parameters.car_following_model) {
            case NAIVE_MODEL:
                return speed + maxAcceleration * TIME_STEP;
            case GIPPS_MODEL:
            case HYBRID_MODEL:
                return getNewSpeedGippsModel();
            case KRAUSS_MODEL:
                return getNewSpeedKraussModel();
            case GFM_MODEL:
                return getNewSpeedGFMModel();
            case IDM_MODEL:
                return getNewSpeedIDMModel();
            case RVF_MODEL:
                return getNewSpeedRVFModel();
            case VFIAC_MODEL:
                return getNewSpeedVFIACModel();
            case OVCM_MODEL:
                return getNewSpeedOVCMModel();
            case KFTM_MODEL:
                return getNewSpeedKFTMModel();
            case HDM_MODEL:
                return getNewSpeedHDMModel();
            case SBM_MODEL:
                return getNewSpeedSBMModel();
            case MY_MODEL:
                return getNewSpeedMyModel();
            default:
                return getNewSpeedGippsModel();
        }
    }

    void tryLaneChange(boolean priorityTowardsMiddle) {
        boolean towardsMiddle = priorityTowardsMiddle == isReverseSegment();
        if (towardsMiddle) {            
            if(!moveToLowerIndexLane()) {
                if(type==PEDESTRIANS_ALONG_THE_ROAD_TYPE || !isObjectInProximity()) moveToHigherIndexLane(); // For a vehicle, it doesn't try to go towards roadside when object is present. For pedestrians, they can try.
            }
        } else {            
            if (!moveToHigherIndexLane()) {
                if(type==PEDESTRIANS_ALONG_THE_ROAD_TYPE || !isObjectInProximity()) moveToLowerIndexLane(); // For a vehicle, it doesn't try to go towards roadside when object is present. For pedestrians, they can try.
            }
        }
    }

    // TODO optimize
    void moveVehicleInSegment() {
        if (type != PEDESTRIANS_ALONG_THE_ROAD_TYPE ) {  
                
            if(isObjectInProximity() || !moveForwardInSegment() || isSlowerVehicleInProximity()) // the order of the three conditions is important and must not be changed
            {
                tryLaneChange(true);
            }     
            
        } else {

            if (Utilities.randInt(0, 101) < Parameters.PEDESTRIAN_RANDOM_LANE_CHANGE_PERCENTAGE) {
                int probability = Parameters.PEDESTRIAN_LEFT_BIAS_PERCENTAGE;
                int randInt = Utilities.randInt(0, 101);
                if (randInt < probability) {
                    if (!isReverseSegment()) {
                        if (!moveToLowerIndexLane()) {
                            if (!moveToHigherIndexLane()) {
                                if(isObjectInProximity())
                                {
                                    tryLaneChange(false);
                                }
                                moveForwardInSegment();
                            }
                        }
                    } else {
                        if (!moveToHigherIndexLane()) {
                            if (!moveToLowerIndexLane()) {
                                if(isObjectInProximity())
                                {
                                    tryLaneChange(false);
                                }
                                moveForwardInSegment();
                            }
                        }
                    }
                }
            } else {
                if(isObjectInProximity())
                {
                    tryLaneChange(false);
                }
                if (!moveForwardInSegment()) {
                    tryLaneChange(false);
                }
            }
        }
    }

    public boolean hasCollided() {

        if (isInIntersection)
            return false;
        return this.getSegment().getStrip(stripIndex).hasCollisionOccurred(this);
    }

    public boolean hasCausedAccident() {
        if (isInIntersection)
            return false;
        Strip strip = link.getSegment(segmentIndex).getStrip(stripIndex);
        return strip.checkForAccident(this);
    }

    public void resetPosition() {
        Vehicle leader = getSegment().getStrip(this.getStripIndex()).getAccidentVehicle(this);
        assert leader != null;

        double leaderDist = leader.getDistanceInSegment();
        distanceInSegment = leaderDist - THRESHOLD_DISTANCE - length;
        speed = 0;
        acceleration = 0;

    }

    public void afterCollisionToDo() {
        if (!hasCollided) {
            this.color = Color.RED;
            hasCollided = true;
            collisionTime = Parameters.simulationStep;
            collisionPenalty = (int) (Utilities.getCollisionPenalty() * 300.0);
            Vehicle leaderCollided = getSegment().getStrip(this.getStripIndex()).getAccidentVehicle(this);
            // leaderCollided can be null when this function is called from the following collider
            if (leaderCollided != null) {
                leaderCollided.afterCollisionToDo();
                Statistics.noOfCollisions++;
                Statistics.noCollisionsPerDemand[demandIndex][type]++;
            }
            speed = 0;
            acceleration = 0;
            penaltyForCollision += Utilities.getCollisionPenalty() * 60.0;

            if (leaderCollided != null) {
                // System.out.println(Parameters.simulationStep + " :: " + vehicleId + " -> " + leaderCollided.getVehicleId());
            }
        }
    }

    public void afterAccidentToDo() {
        if (!hasCollided) {
            this.color = Color.RED;
            hasCollided = true;
            collisionTime = Parameters.simulationStep;
            collisionPenalty = (int) (Utilities.getCollisionPenalty() * 300.0);
            speed = 0;
            acceleration = 0;
            penaltyForCollision += Utilities.getCollisionPenalty() * 60.0;

            Statistics.noOfAccidents++;
            Statistics.noAccidentsPerDemand[demandIndex][type]++;
        }
    }

    void printAccidentLog() {
        if (!hasCollided) {
            Vehicle leader = this.getSegment().getStrip(this.getStripIndex()).getAccidentVehicle(this);
            int leaderType = leader == null ? -1 : leader.getType();
            double leaderSpeed = leader == null ? NaN : leader.getSpeed();
            double leaderAcc = leader == null ? NaN : leader.getAcceleration();
            try (PrintWriter writer = new PrintWriter(new FileOutputStream("statistics/accident_log.csv", true))) {
                // sim_step, vehicle_id, type, speed, leader_type, leader_speed, acceleration, collision_penalty
                writer.printf("%d, %d, %d, %.3f, %.3f, %d, %.3f, %.3f", Parameters.simulationStep, this.getVehicleId(), this.getType(),
                        this.getSpeed(), this.getAcceleration(), leaderType, leaderSpeed, leaderAcc);
                writer.println();
            } catch (FileNotFoundException e) {
                e.printStackTrace();
            }
        }
    }

    private boolean controlSpeedInSegment() {
        storePrevSpeeds();
        speed = getNewSpeed();
        if (speed > currentMaxSpeed) {
            speed = currentMaxSpeed;
        }
//        checkCrashCondition(speed);
//        if (Parameters.car_following_model == Parameters.CAR_FOLLOWING_MODEL.NAIVE_MODEL || Parameters.car_following_model == Parameters.CAR_FOLLOWING_MODEL.HYBRID_MODEL) {
        if (Parameters.BRAKE_HARD) {
            double gap = speed * TIME_STEP + 0.5 * maxAcceleration * TIME_STEP * TIME_STEP;
//            double gap = speed * TIME_STEP;
            for (int i = stripIndex; i < stripIndex + numberOfStrips; i++) {
                Segment segment = link.getSegment(segmentIndex);
                Strip strip = segment.getStrip(i);
                double stripGap = strip.getGapForForwardMovement(this);
                if (stripGap == 0) {
                    speed = 0;
                    return false;
                } else {
                    gap = min(gap, stripGap);
                }
            }
            speed = gap / TIME_STEP;
            return true;
        } else {
            return speed > 0;
        }
    }

    private boolean isMoveForwardPossible() {
        double storeSpeed = speed;
        speed = getNewSpeed();
        if (speed > currentMaxSpeed) {
            speed = currentMaxSpeed;
        }
//        if (Parameters.car_following_model == Parameters.CAR_FOLLOWING_MODEL.NAIVE_MODEL || Parameters.car_following_model == Parameters.CAR_FOLLOWING_MODEL.HYBRID_MODEL) {
        if (Parameters.BRAKE_HARD) {
            for (int i = stripIndex; i < stripIndex + numberOfStrips; i++) {
                Segment segment = link.getSegment(segmentIndex);
                Strip strip = segment.getStrip(i);
                double stripGap = strip.getGapForForwardMovement(this);
                if (stripGap == 0) {
                    speed = storeSpeed;
                    return false;
                }
            }
            speed = storeSpeed;
            return true;
        } else {
            boolean res = speed > 0;
            speed = storeSpeed;
            return res;
        }
    }

    void printVehicleDetails() {
        if (Parameters.DEBUG_MODE) {
            if (vehicleId == 1767|| vehicleId == 109 || vehicleId == -10 || vehicleId == -110) {
                String pathname = "debug/debug" + vehicleId + ".txt";
                try (PrintWriter writer = new PrintWriter(new FileOutputStream(new File(pathname), true))) {
                    writer.println("Sim step: " + Parameters.simulationStep);
                    writer.println("Vehicle ID: " + vehicleId);
                    writer.println("Type: " + type);
                    writer.printf("Speed: %.2f\n", speed);
                    writer.printf("Acceleration: %.2f\n", acceleration);
                    writer.printf("Length: %.2f\n", length);
                    writer.println("Accident Time: " + collisionTime);
                    if (isInIntersection) {
                        writer.println("Intersection Mode: " + true);
                        writer.println("Intersection ID: " + node.getId());
                        writer.printf("Distance in Intersection: %.2f\n", distanceInIntersection);
                        writer.printf("Intersection Length: %.2f\n", node.getIntersectionStrip(intersectionStripIndex).getLength() / Parameters.pixelPerMeter);
                    } else {
                        writer.println("Link ID: " + link.getId());
                        writer.println("Segment Index: " + segmentIndex);
                        writer.println("Strip Index: " + stripIndex);
                        Vehicle pLeader = getProbableLeader();
                        Vehicle pFollower = getProbableFollower();
                        int leaderId = pLeader == null ? -1 : pLeader.vehicleId;
                        int followerId = pFollower == null ? -1 : pFollower.vehicleId;
                        writer.println("Leader: " + leaderId);
                        writer.println("Follower: " + followerId);
                        writer.printf("Distance in Segment: %.2f\n", distanceInSegment);
                        writer.printf("Segment Length: %.2f\n", link.getSegment(segmentIndex).getLength());
                        writer.printf("Segment Width: %.2f\n", link.getSegment(segmentIndex).getSegmentWidth());
                        writer.println("Reverse Link: " + reverseLink);
                        writer.println("Reverse Segment: " + reverseSegment);
                    }
                    writer.println();
                } catch (FileNotFoundException e) {
                    e.printStackTrace();
                }
            }

        }
    }

    int getStartStripForMyModel(int deltaIndex){
        int startIndex = stripIndex;
        if (type > -12) {
            if (reverseSegment) {
                startIndex = max(startIndex - deltaIndex, getSegment().middleHighStripIndex);
            } else {
                startIndex = max(1, startIndex - deltaIndex);
            }
        }
        return startIndex;
    }

    int getEndStripForMyModel(int deltaIndex) {
        int endIndex = stripIndex + numberOfStrips - 1;
        if (type != PEDESTRIANS_ALONG_THE_ROAD_TYPE) {
            if (reverseSegment) {
                endIndex = min(endIndex + deltaIndex, getSegment().lastVehicleStripIndex);
            } else {
                endIndex = min(endIndex + deltaIndex, getSegment().middleLowStripIndex);
            }
        }
        return endIndex;
    }

    Vehicle getMyProbableLeader(int sideStripsToConsider) {
        Vehicle leader = null;
        int startIndex = getStartStripForMyModel(sideStripsToConsider);
        int endIndex = getEndStripForMyModel(sideStripsToConsider);
        for (int i = startIndex; i <= endIndex; i++) {
            Segment segment = link.getSegment(segmentIndex);
            Strip strip = segment.getStrip(i);
            Vehicle v = strip.getProbableLeaderForMyModel(this);
            if (v != null) {
                if (leader == null) {
                    leader = v;
                } else {
                    if (leader.getDistanceInSegment() > v.getDistanceInSegment()) {
                        leader = v;
                    }
                }
            }
        }
        return leader;
    }

    /**
     * @return the vehicle closest and in front of this among all the strips (null if none is in front of it)
     */
    Vehicle getProbableLeader() {
        Vehicle leader = null;
        for (int i = stripIndex; i < stripIndex + numberOfStrips; i++) {
            Segment segment = link.getSegment(segmentIndex);
            Strip strip = segment.getStrip(i);
            Vehicle v = strip.getProbableLeaderForMyModel(this);
            if (v != null) {
                if (leader == null) {
                    leader = v;
                } else {
                    if (leader.getDistanceInSegment() > v.getDistanceInSegment()) {
                        leader = v;
                    }
                }
            }
        }
        return leader;
    }

    Object getProbableObjectLeader() {

        Object leader = null;
        for (int i = stripIndex; i < stripIndex + numberOfStrips; i++) {
            Segment segment = link.getSegment(segmentIndex);
            Strip strip = segment.getStrip(i);
            Object o = strip.probableObjectLeader(this);
            if (o != null) {
                if (leader == null) {
                    leader = o;
                } else {
                    if (leader.getDistanceInSegment() > o.getDistanceInSegment()) {
                        leader = o;
                    }
                }
            }
        }
        return leader;
    }


    Vehicle getProbableFollower() {
        Vehicle follower = null;
        for (int i = stripIndex; i < stripIndex + numberOfStrips; i++) {
            Segment segment = link.getSegment(segmentIndex);
            Strip strip = segment.getStrip(i);
            Vehicle v = strip.probableFollower(this);
            if (v != null) {
                if (follower == null) {
                    follower = v;
                } else {
                    if (follower.getDistanceInSegment() + follower.length < v.getDistanceInSegment() + v.length) {
                        follower = v;
                    }
                }
            }
        }
        return follower;
    }


    ArrayList<Vehicle> getAllLeadersForMyModel(int sideStripsToConsider) {
        ArrayList<Vehicle> leaders = new ArrayList<>();
        for (int i = getStartStripForMyModel(sideStripsToConsider); i <= getEndStripForMyModel(sideStripsToConsider); i++) {
            Segment segment = link.getSegment(segmentIndex);
            Strip strip = segment.getStrip(i);
            Vehicle v = strip.getProbableLeaderForMyModel(this);
            if (v != null) {
                leaders.add(v);
            }
        }
        return leaders;
    }

    void printAllLeaders() {
        ArrayList<Vehicle> leaders = getAllLeadersForMyModel(Parameters.SIDE_STRIPS_TO_CONSIDER);
        if (vehicleId == 4) {
            System.out.printf("This: %d; Leaders: ", this.vehicleId);
            for (Vehicle leader : leaders) {
                double w = Utilities.getWeight(leader, this, Parameters.SIDE_STRIPS_TO_CONSIDER);
                System.out.printf("%d %.2f\t", leader.vehicleId, w);
            }
            System.out.println();
        }
    }

    private boolean isSlowerVehicleInProximity() {
        Vehicle leader = getProbableLeader();
        if (leader != null) {
            if (this.type == PEDESTRIANS_ALONG_THE_ROAD_TYPE) {
                return leader.currentMaxSpeed < this.currentMaxSpeed
                        && leader.getDistanceInSegment() < this.getDistanceInSegment() + length + this.currentMaxSpeed;
            } else {
                return switch (this.dlcModel) {
                    case NAIVE_MODEL -> leader.currentMaxSpeed < this.currentMaxSpeed
                            && leader.getDistanceInSegment() < this.getDistanceInSegment() + length + this.currentMaxSpeed;
//                    case GHR_MODEL -> getAccelerationGHRModel(leader, this) < 0;
                    default -> getGap(leader, this) < getDistanceForDesiredSpeed(leader, this);
                };
            }

        } else {
            // no leader
            return false;
        }
    }

    public boolean isObjectInProximity()
	{
		
        Object objectLeader = getProbableObjectLeader();
        double threshold_distance = 2;

        if(objectLeader!=null)
        {
            if(this.type == PEDESTRIANS_ALONG_THE_ROAD_TYPE)
            {
                return objectLeader.getDistanceInSegment() < this.getDistanceInSegment() + length + this.currentMaxSpeed + threshold_distance;
            }
            else
            {

                // return objectLeader.getDistanceInSegment() < this.getDistanceInSegment() + length + this.currentMaxSpeed + threshold_distance;
                // have to fix for other models, currently uses naive model
                return switch (this.dlcModel) {
                    case NAIVE_MODEL -> objectLeader.getDistanceInSegment() < this.getDistanceInSegment() + length + this.currentMaxSpeed + threshold_distance;
                    case GHR_MODEL -> getAccelerationGHRModel(leader, this) < 0;
                    default -> getObjectGap(objectLeader, this) < getObjectDistanceForDesiredSpeed(this);
                };
            }
        } 
        else
        {
            return false;
        }
	}

    /**
     * does not change the distance in intersection
     * just change the speed if necessary
     *
     * @return whether movement is possible or not
     */
    @SuppressWarnings("Duplicates")
    private boolean controlSpeedInIntersection() {
        double storeDistanceInIntersection = distanceInIntersection;
        speed += maxAcceleration * TIME_STEP;

        // this block is for making a small jump while deadlocked in intersection
        if (!noForceMove) {
            speed += length * 1.5;
        }

        if (speed > currentMaxSpeed) {
            speed = currentMaxSpeed;
        }
        double storeSpeed = speed;

        double tempSpeed = 0;
        tempSpeed += maxAcceleration * TIME_STEP;
        boolean movementPossible = false;

        while (tempSpeed <= storeSpeed) {
            speed = tempSpeed;
            distanceInIntersection += tempSpeed * TIME_STEP;
            if (node.doOverlap(this) && noForceMove) {
                distanceInIntersection = storeDistanceInIntersection;
                tempSpeed -= maxAcceleration * TIME_STEP;
                speed = tempSpeed;
                return movementPossible;
            }
            movementPossible = true;
            tempSpeed += maxAcceleration * TIME_STEP;
            distanceInIntersection = storeDistanceInIntersection;
        }
        distanceInIntersection = storeDistanceInIntersection;
        return true;
    }

    private boolean moveForwardInSegment() {
        Segment segment = link.getSegment(segmentIndex);
        if (controlSpeedInSegment()) {
//            modifySpeedAndAcc();
            storePrevGaps();
            distanceInSegment = getNewDistanceInSegment();
            if (distanceInSegment > segment.getLength() - length) { // distance in segment is the tail of the vehicle so vehicle length is subtracted from segment length
                distanceInSegment = segment.getLength() - length - (MARGIN - 0.1);   // here MARGIN is the margin, slightly less than the margin in isSegmentEnd function
            }
            if (!passedSensor) {
                if (distanceInSegment > segment.getSensor()) {
                    passedSensor = true;
                }
            }
            return true;
        } else {
            waitingTime++;
            waitingTimeInSegment++;
            return false;
        }
    }

    private void modifySpeedAndAcc() {
        double storeDistInSegment = distanceInSegment;
        double newDistInSegment = getNewDistanceInSegment();
        if (newDistInSegment > link.getSegment(segmentIndex).getLength() - length) {
            newDistInSegment = link.getSegment(segmentIndex).getLength() - length - (MARGIN - 0.1);
            double deltaDistance = newDistInSegment - storeDistInSegment;
            double modifiedSpeed = deltaDistance / TIME_STEP;
            acceleration = (modifiedSpeed - speed) / TIME_STEP;
            speed = modifiedSpeed;
        }
    }

    private void printEndPoints(int index, Point2D p) {
        if (vehicleId == 86 || vehicleId == 186) {
            String pathname = "debug_ep" + vehicleId + ".txt";
            try (PrintWriter writer = new PrintWriter(new FileOutputStream(new File(pathname), true))) {
                writer.println("Index: " + index + " End Point: " + p.x + " " + p.y);
            } catch (FileNotFoundException e) {
                e.printStackTrace();
            }
        }
    }

    /**
     * try to move forward by changing the end point i.e. direction of the intersection strip
     *
     * @param newStripIndex calculate end point from this index on the entering segment of the intersection strip
     * @return by changing into new direction whether we can move forward or not
     */
    private boolean isChangeDirectionInIntersectionFruitful(int newStripIndex) {
        IntersectionStrip is = getCurrentIntersectionStrip();
        int storeNewStripIndex = is.endStrip;
        double storeEndPointX = is.endPointX;
        double storeEndPointY = is.endPointY;

        Point2D newEndPoint = Utilities.getNewEndPointForIntersectionStrip(this, is, newStripIndex);
        is.endPointX = newEndPoint.x;
        is.endPointY = newEndPoint.y;
        is.endStrip = newStripIndex;
        if (isMoveForwardPossibleInIntersection()) {
            return true;
        } else {
            is.endStrip = storeNewStripIndex;
            is.endPointX = storeEndPointX;
            is.endPointY = storeEndPointY;
            return false;
        }
    }

    /**
     * Changes nothing. Just checks if move forward is possible. Similar to controlSpeedInIntersection
     *
     * @return move forward possible or not
     */
    @SuppressWarnings("Duplicates")
    private boolean isMoveForwardPossibleInIntersection() {
        double storeDistanceInIntersection = distanceInIntersection;
        double storeInitialSpeed = speed;

        speed += maxAcceleration * TIME_STEP;
        if (speed > currentMaxSpeed) {
            speed = currentMaxSpeed;
        }
        double storeSpeed = speed;

        double tempSpeed = 0;
        tempSpeed += maxAcceleration;
        boolean movementPossible = false;

        while (tempSpeed <= storeSpeed) {
            speed = tempSpeed;
            distanceInIntersection += tempSpeed;
            if (node.doOverlap(this)) {
                distanceInIntersection = storeDistanceInIntersection;
                speed = storeInitialSpeed;
                return movementPossible;
            }
            movementPossible = true;
            tempSpeed += maxAcceleration;
            distanceInIntersection = storeDistanceInIntersection;
        }
        distanceInIntersection = storeDistanceInIntersection;
        speed = storeInitialSpeed;
        return true;
    }

    /**
     * try all the strips of the entering segment to enter when stuck in current direction
     * returns whether it can move forward or not after changing direction
     */
    private boolean tryChangingDirectionInIntersection() {
//        System.out.printf("Vehicle: %d stuck in node: %d at time %d %n", vehicleId, node.getId(), simulationStep);
        IntersectionStrip is = getCurrentIntersectionStrip();
        int beginLimit, endLimit;
        if (is.endStrip <= is.enteringSegment.middleLowStripIndex) {
            beginLimit = 1;
            endLimit = is.enteringSegment.middleLowStripIndex - (numberOfStrips - 1);
        } else {
            beginLimit = is.enteringSegment.middleHighStripIndex;
            endLimit = is.enteringSegment.lastVehicleStripIndex - (numberOfStrips - 1);
        }
        for (int i = beginLimit; i <= endLimit; i++) {
            if (isChangeDirectionInIntersectionFruitful(i)) {
                stuckInIntersection = 0;
                return true;
            }
        }
        return false;
    }

    private boolean isSlowerVehicleInProximityInIntersection() {
        double storeDistanceInIntersection = distanceInIntersection;
        distanceInIntersection += speed * TIME_STEP;
        if (node.doOverlap(this)) {
            // overlaps when moves forward in full speed; so vehicle in proximity
            Vehicle obstructingVehicle = node.getOverlappingVehicle(this);
            if (obstructingVehicle.speed < this.speed) {
                // obstructing vehicle is slower
                distanceInIntersection = storeDistanceInIntersection;
                return true;
            }
            // obstructing vehicle is not slow
        }
        // no vehicle in proximity
        distanceInIntersection = storeDistanceInIntersection;
        return false;
    }

    void moveVehicleInIntersection() {
        if (!moveForwardInIntersection() || isSlowerVehicleInProximityInIntersection()) {
            tryChangingDirectionInIntersection();
        }
    }

    private boolean moveForwardInIntersection() {
        if (controlSpeedInIntersection()) {
            distanceInIntersection += speed * TIME_STEP;
            double pseudoLength = node.getIntersectionStrip(intersectionStripIndex).getLength() / Parameters.pixelPerMeter;
            if (distanceInIntersection > pseudoLength - length) {
                distanceInIntersection = pseudoLength - length - (MARGIN - 0.1);
            }
            stuckInIntersection = 0;
            noForceMove = false;
            return true;
        } else {
            stuckInIntersection++;
            noForceMove = stuckInIntersection <= 40;
            waitingTime++;
            return false;
        }
    }

    boolean isAtSegmentEnd() {
        Segment segment = link.getSegment(segmentIndex);
        return distanceInSegment + length + MARGIN >= segment.getLength();
    }

    boolean isAtIntersectionEnd() {
        IntersectionStrip intersectionStrip = node.getIntersectionStrip(intersectionStripIndex);
        return (distanceInIntersection + length + MARGIN) * Parameters.pixelPerMeter >= intersectionStrip.getLength();
    }

    /**
     * oldSegmentWidth != newSegmentWidth; So we need to calculate new strip index
     *
     * @param leavingSegment  = current/leaving segment
     * @param enteringSegment = entering segment
     * @param isReverse       = whether the entering segment is reverse or not
     * @return strip index in the new link
     */
    int getNewStripIndex(Segment leavingSegment, Segment enteringSegment, boolean isReverse) {
        int newStripIndex;
        int stripIndexForVehicle, oldLimit, newLimit;

        if (stripIndex >= leavingSegment.middleHighStripIndex) {
            stripIndexForVehicle = stripIndex - leavingSegment.middleHighStripIndex + 1;
            oldLimit = leavingSegment.lastVehicleStripIndex - leavingSegment.middleHighStripIndex + 1;
        } else {
            stripIndexForVehicle = stripIndex;
            oldLimit = leavingSegment.middleLowStripIndex;
        }

        if (isReverse) {
            newLimit = enteringSegment.lastVehicleStripIndex - enteringSegment.middleHighStripIndex + 1;
            newStripIndex = (int) round(1.0 * stripIndexForVehicle / oldLimit * newLimit);

            if (newStripIndex == 0) {
                newStripIndex++;
            }

            if (!isReverseSegment()) {
                newStripIndex = newLimit - newStripIndex + 1;
            }

            newStripIndex = enteringSegment.middleHighStripIndex + newStripIndex - 1;

            if (enteringSegment.lastVehicleStripIndex - newStripIndex + 1 < numberOfStrips) {
                newStripIndex = enteringSegment.lastVehicleStripIndex - numberOfStrips + 1;
            }

            if (!(newStripIndex >= enteringSegment.middleHighStripIndex && (newStripIndex + numberOfStrips - 1) <= enteringSegment.lastVehicleStripIndex)) {
                System.out.println("Reverse: >>>>>>>>>>>>>>>>>" + vehicleId);
                assert false;  // if the road is not wide enough then it can be here
            }
        } else {
            newLimit = enteringSegment.middleLowStripIndex;
            newStripIndex = (int) round(1.0 * stripIndexForVehicle / oldLimit * newLimit);

            if (newStripIndex == 0) {
                newStripIndex++;
            }

            if (isReverseSegment()) {
                newStripIndex = newLimit - newStripIndex + 1;
            }

            if (enteringSegment.middleLowStripIndex - newStripIndex + 1 < numberOfStrips) {
                newStripIndex = enteringSegment.middleLowStripIndex - numberOfStrips + 1;
            }

            if (!(newStripIndex >= 1 && (newStripIndex + numberOfStrips - 1) <= enteringSegment.middleLowStripIndex)) {
                System.out.println("Straight: >>>>>>>>>>>>>" + vehicleId);
                assert false; // if the road is not wide enough then it can be here
            }
        }

        return newStripIndex;
    }

    boolean hasPenaltyTimePassed() {
        if ((Parameters.simulationStep - collisionTime) >= penaltyForCollision) {
            return true;
        } else {
            return false;
        }
    }

    //this function is called when the trip is completed or it has caused an accident
    void removeFromSimulation(boolean hasCompletedTrip){
        if(hasCompletedTrip == false)System.out.println("mara");
        this.setToRemove(true);
        this.freeStrips();
        this.decreaseVehicleCountOnSegment();
        if (hasCompletedTrip) {
            this.updateSegmentLeavingData();
        }
    }

    public boolean isHasAlreadyCollided() {
        return hasCollided;
    }

    IntersectionStrip getCurrentIntersectionStrip() {
        return node.getIntersectionStrip(intersectionStripIndex);
    }

    double getThresholdDistance() {
        return THRESHOLD_DISTANCE + SAFE_TIME_GAP * speed; // here 1.8 is the desired time gap
    }

    double getMaxBraking() {
        return maxBraking;
    }

    public SIGNAL getSignalOnLink() {
        return signalOnLink;
    }

    public void setSignalOnLink(SIGNAL signalOnLink) {
        this.signalOnLink = signalOnLink;
    }

    public int getVehicleId() {
        return vehicleId;
    }

    public int getType() {
        return type;
    }

    public void setType(int type) {
        this.type = type;
    }

    public double getLength() {
        return length;
    }

    public void setLength(double length) {
        this.length = length;
    }

    public double getWidth() {
        return width;
    }

    public void setWidth(double width) {
        this.width = width;
    }

    int getNumberOfStrips() {
        return numberOfStrips;
    }

    public void setNumberOfStrips(int numberOfStrips) {
        this.numberOfStrips = numberOfStrips;
    }

    public double getSpeed() {
        return speed;
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public double getAcceleration() {
        return acceleration;
    }

    public void setAcceleration(double acceleration) {
        this.acceleration = acceleration;
    }

    public Color getColor() {
        return color;
    }

    public void setColor(Color color) {
        this.color = color;
    }

    int getDemandIndex() {
        return demandIndex;
    }

    public void setDemandIndex(int demandIndex) {
        this.demandIndex = demandIndex;
    }

    int getPathIndex() {
        return pathIndex;
    }

    public void setPathIndex(int pathIndex) {
        this.pathIndex = pathIndex;
    }

    int getLinkIndexOnPath() {
        return linkIndexOnPath;
    }

    public void setLinkIndexOnPath(int linkIndexOnPath) {
        this.linkIndexOnPath = linkIndexOnPath;
    }

    public double getSegStartX() {
        return segmentStartPoint.x;
    }

    public void setSegStartX(double segStartX) {
        this.segmentStartPoint.x = segStartX;
    }

    public double getSegStartY() {
        return segmentStartPoint.y;
    }

    public void setSegStartY(double segStartY) {
        this.segmentStartPoint.y = segStartY;
    }

    public double getSegEndX() {
        return segmentEndPoint.x;
    }

    public void setSegEndX(double segEndX) {
        this.segmentEndPoint.x = segEndX;
    }

    public double getSegEndY() {
        return segmentEndPoint.y;
    }

    void setSegEndY(double segEndY) {
        this.segmentEndPoint.y = segEndY;
    }

    boolean isInIntersection() {
        return isInIntersection;
    }

    void setInIntersection(boolean inIntersection) {
        this.isInIntersection = inIntersection;
    }

    boolean isReverseLink() {
        return reverseLink;
    }

    void setReverseLink(boolean reverseLink) {
        this.reverseLink = reverseLink;
    }

    boolean isReverseSegment() {
        return reverseSegment;
    }

    void setReverseSegment(boolean reverseSegment) {
        this.reverseSegment = reverseSegment;
    }

    boolean isPassedSensor() {
        return passedSensor;
    }

    public void setPassedSensor(boolean passedSensor) {
        this.passedSensor = passedSensor;
    }

    public boolean isToRemove() {
        return toRemove;
    }

    public void setToRemove(boolean toRemove) {
        this.toRemove = toRemove;
    }

    public Link getLink() {
        return link;
    }

    public void setLink(Link link) {
        this.link = link;
    }

    public int getSegmentIndex() {
        return segmentIndex;
    }

    public void setSegmentIndex(int segmentIndex) {
        this.segmentIndex = segmentIndex;
    }

    public int getStripIndex() {
        return stripIndex;
    }

    public double getDistanceInSegment() {
        return distanceInSegment;
    }

    public void setDistanceInSegment(double distanceInSegment) {
        this.distanceInSegment = distanceInSegment;
    }

    private double getDistanceFromSegmentEnd() {
        return getSegment().getLength() - distanceInSegment - length - MARGIN;
    }

    public Node getNode() {
        return node;
    }

    public void setNode(Node node) {
        this.node = node;
    }

    int getIntersectionStripIndex() {
        return intersectionStripIndex;
    }

    void setIntersectionStripIndex(int intersectionStripIndex) {
        this.intersectionStripIndex = intersectionStripIndex;
    }

    public double getDistanceInIntersection() {
        return distanceInIntersection;
    }

    void setDistanceInIntersection(double distanceInIntersection) {
        this.distanceInIntersection = distanceInIntersection;
    }

    public Vehicle getLeader() {
        return leader;
    }

    public void setLeader(Vehicle leader) {
        this.leader = leader;
    }

    Point2D[] getSegmentCorners() {
        return segmentCorners;
    }

    void calculateCornerPoints() {
        double x1, x2, x3, x4;
        double y1, y2, y3, y4;
        double x_1 = node.getIntersectionStrip(intersectionStripIndex).startPointX;
        double x_2 = node.getIntersectionStrip(intersectionStripIndex).endPointX;
        double y_1 = node.getIntersectionStrip(intersectionStripIndex).startPointY;
        double y_2 = node.getIntersectionStrip(intersectionStripIndex).endPointY;
        double p_s_l = Math.hypot(x_1 - x_2, y_1 - y_2);
        double l = getLength() * Parameters.pixelPerMeter;
        double d_i_j = distanceInIntersection * Parameters.pixelPerMeter;
        double t1 = d_i_j / p_s_l;
        x1 = (1 - t1) * x_1 + t1 * x_2;
        y1 = (1 - t1) * y_1 + t1 * y_2;
        double t2 = (d_i_j + l) / p_s_l;
        x2 = (1 - t2) * x_1 + t2 * x_2;
        y2 = (1 - t2) * y_1 + t2 * y_2;
        double w = getWidth() * Parameters.pixelPerMeter;
        x3 = Utilities.returnX3(x1, y1, x2, y2, w);
        y3 = Utilities.returnY3(x1, y1, x2, y2, w);
        x4 = Utilities.returnX4(x1, y1, x2, y2, w);
        y4 = Utilities.returnY4(x1, y1, x2, y2, w);
        segmentCorners[0] = new Point2D(x1, y1);
        segmentCorners[1] = new Point2D(x2, y2);
        segmentCorners[2] = new Point2D(x4, y4);
        segmentCorners[3] = new Point2D(x3, y3);
    }

    /**
     * this function updates required statistical info before leaving a segment
     */
    void updateSegmentLeavingData() {
        setSegmentLeaveTime(Parameters.simulationStep);
        increaseTraveledDistance(getSegment().getLength());
        getSegment().updateAvgSpeedInSegment(getAvgSpeedInSegment());
        getSegment().increaseTotalWaitingTime((int) (waitingTimeInSegment * Constants.TIME_STEP));
    }

    /**
     * this function updates required statistical info before entering a segment
     */
    void updateSegmentEnteringData() {
        waitingTimeInSegment = 0;
        setSegmentEnterTime(Parameters.simulationStep);
        getSegment().increaseEnteringVehicleCount();
        this.setSpeed(this.getNewSpeed());
    }

    private double getAvgSpeedInSegment() {
        return getSegment().getLength() / ((segmentLeaveTime - segmentEnterTime) * Constants.TIME_STEP);
    }

    private void setSegmentEnterTime(int segmentEnterTime) {
        this.segmentEnterTime = segmentEnterTime;
    }

    private void setSegmentLeaveTime(int segmentLeaveTime) {
        this.segmentLeaveTime = segmentLeaveTime;
    }

    int getWaitingTime() {
        return (int) (waitingTime * Constants.TIME_STEP);
    }

    int getTravelTime() {
        if (endTime == 0) {
            endTime = Parameters.simulationStep;
        }
        return (int) ((endTime - startTime) * Constants.TIME_STEP);
    }

    public double getDistanceTraveled() {
        return distanceTraveled;
    }

    /**
     * this function computes the fuel consumed in each TIME_STEP
     *
     * @return the consumed fuel in current SIMULATION_STEP in litre/TIME_STEP
     */
    double computeFuelConsumption() {
        double mass = 1325.0; // kg
        double t = TIME_STEP;
        double d_petrol = 0.73722; // gm/cc at 60 F
        double fIdle = 0.299; // gm/s

        double alpha = 0.365;
        double beta = 0.00114;
        double delta = 9.65 / 10000000;
        double xita = 0.0943;
        double A = 0.1326;        // for tires.
        double B = 0.0027384; //274.4;%205.8;%823.2;
        double C = 0.0010843;
        double v = speed * 3600 / 1000; // km/hr
        double Rt = A * v + B * (v * v) + C * (v * v * v) + mass * acceleration * v;
        double av = acceleration * speed;

        double delF;
        if (Rt > 0) {
            delF = alpha + beta * v + delta * (v * v * v) + xita * av;
        } else {
            delF = fIdle;
        }

        return delF / d_petrol * t / 1000; // litre/TIME_STEP
    }

    public void incrementFuelConsumption() {
        fuelConsumption += computeFuelConsumption();
    }

    void updateTripTimeStatistics() {
        Statistics.noOfVehiclesCompletingTrip[demandIndex][type]++;
        Statistics.tripTime[demandIndex][type] += getTravelTime();
        Statistics.tripTime[demandIndex][type] += penaltyForCollision;
        Statistics.totalFuelConsumption[demandIndex][type] += fuelConsumption;
        // if(demandIndex == 0 || demandIndex == 1)System.out.println("distance from 2 to 9: "+distanceTraveled);

    }

    /**
     * this function is used to calculate and update various statistical results
     * when a car finishes its trip or the whole simulation finish
     */
    void calculateStatisticsAtEnd() {
        endTime = Parameters.simulationStep;
        int travelTime = endTime - startTime;
        double avgSpeed = distanceTraveled / travelTime;
        Statistics.noOfVehicles[type]++;
        Statistics.avgSpeedOfVehicle[type] += avgSpeed;
        Statistics.waitingTime[type] += waitingTime;
        Statistics.totalTravelTime[type] += travelTime;
        // if(demandIndex == 0 || demandIndex == 1)System.out.println("distance from 2 to 9: "+distanceTraveled);
    }

    void increaseTraveledDistance(double amount) {
        distanceTraveled += amount;
    }

    private void checkCrashCondition(double currentSpeed) {
        Vehicle leader = getProbableLeader();
        if (leader != null) {
            double gap = leader.distanceInSegment - distanceInSegment - length;
            double leaderSpeed = leader.vehicleId < vehicleId ? leader.getSpeed() : leader.getNewSpeed();
            if (currentSpeed > leaderSpeed) {
                double TTC = gap / (currentSpeed - leaderSpeed);

                if (TTC <= Parameters.TTC_THRESHOLD) {
                    getSegment().increaseNearCrashCount();
                }
            }

        }
    }

    Segment getSegment() {
        return getLink().getSegment(segmentIndex);
    }

    void segmentChange(int segmentIndex, int stripIndex) {
        this.segmentIndex = segmentIndex;
        this.stripIndex = stripIndex;

        this.distanceInSegment = 0.1;
        this.passedSensor = false;
        occupyStrips();
        increaseVehicleCountOnSegment();
    }

    void linkChange(int linkIndexOnPath, Link link, int segmentIndex, int stripIndex) {
        this.linkIndexOnPath = linkIndexOnPath;
        this.link = link;
        segmentChange(segmentIndex, stripIndex);
    }

    @SuppressWarnings("Duplicates")
    void drawVehicle(BufferedWriter traceWriter, Graphics g, double pixelPerStrip, double pixelPerMeter, double pixelPerFootPathStrip) {
        int x1, x2, x3, x4, y1, y2, y3, y4;
        int[] xs;
        int[] ys;

        if (isInIntersection) {
            IntersectionStrip intersectionStrip = node.getIntersectionStrip(intersectionStripIndex);
            double intersectionStripLength = intersectionStrip.getLength();

            int length = (int) (getLength() * pixelPerMeter);

            double dis = distanceInIntersection * pixelPerMeter;
            //Using internally section or ratio formula,it finds the coordinates along which vehicles are
            double xp = (dis * intersectionStrip.endPointX + (intersectionStripLength - dis) * intersectionStrip.startPointX) / intersectionStripLength;// * mpRatio;
            double yp = (dis * intersectionStrip.endPointY + (intersectionStripLength - dis) * intersectionStrip.startPointY) / intersectionStripLength;// * mpRatio;
            double xq = ((dis + length) * intersectionStrip.endPointX + (intersectionStripLength - (dis + length)) * intersectionStrip.startPointX) / intersectionStripLength;// * mpRatio;
            double yq = ((dis + length) * intersectionStrip.endPointY + (intersectionStripLength - (dis + length)) * intersectionStrip.startPointY) / intersectionStripLength;// * mpRatio;

            x1 = (int) Math.round(xp);
            y1 = (int) Math.round(yp);
            x2 = (int) Math.round(xq);
            y2 = (int) Math.round(yq);

            int width = (int) Math.round(this.width * pixelPerMeter);

            //finds the coordinates of perpendicularly opposite lower(right) points
            if (reverseSegment) {
                x3 = (int) Math.round(Utilities.returnX5(x1, y1, x2, y2, width));
                y3 = (int) Math.round(Utilities.returnY5(x1, y1, x2, y2, width));
                x4 = (int) Math.round(Utilities.returnX6(x1, y1, x2, y2, width));
                y4 = (int) Math.round(Utilities.returnY6(x1, y1, x2, y2, width));
            } else {
                x3 = (int) Math.round(Utilities.returnX3(x1, y1, x2, y2, width));
                y3 = (int) Math.round(Utilities.returnY3(x1, y1, x2, y2, width));
                x4 = (int) Math.round(Utilities.returnX4(x1, y1, x2, y2, width));
                y4 = (int) Math.round(Utilities.returnY4(x1, y1, x2, y2, width));
            }
            xs = new int[]{x1, x2, x4, x3};
            ys = new int[]{y1, y2, y4, y3};
        } else {

            Segment segment = link.getSegment(segmentIndex);
            double segmentLength = segment.getLength();
            double marginStrip = (getNumberOfStrips() * Parameters.stripWidth - width) / 2.0;

            //--->strategy: if its source then change it to int right away
            //int length = (int) Math.ceil(getLength());
            //Using internally section or ratio formula,it finds the coordinates along which vehicles are
            double xp = (getDistanceInSegment() * segmentEndPoint.x + (segmentLength - getDistanceInSegment()) * segmentStartPoint.x) / segmentLength * pixelPerMeter;
            double yp = (getDistanceInSegment() * segmentEndPoint.y + (segmentLength - getDistanceInSegment()) * segmentStartPoint.y) / segmentLength * pixelPerMeter;
            double xq = ((getDistanceInSegment() + length) * segmentEndPoint.x + (segmentLength - (getDistanceInSegment() + length)) * segmentStartPoint.x) / segmentLength * pixelPerMeter;
            double yq = ((getDistanceInSegment() + length) * segmentEndPoint.y + (segmentLength - (getDistanceInSegment() + length)) * segmentStartPoint.y) / segmentLength * pixelPerMeter;

            double w = 1 * pixelPerFootPathStrip + (stripIndex - 1) * pixelPerStrip + marginStrip * pixelPerMeter;
            //finds the coordinates of vehicles starting and ending upper points depending on which strip their upper(left) portion are.
            if (reverseSegment) {
                x1 = (int) Math.round(Utilities.returnX5(xp, yp, xq, yq, w));
                y1 = (int) Math.round(Utilities.returnY5(xp, yp, xq, yq, w));
                x2 = (int) Math.round(Utilities.returnX6(xp, yp, xq, yq, w));
                y2 = (int) Math.round(Utilities.returnY6(xp, yp, xq, yq, w)); //(v.getStrip().getStripIndex())*stripPixelCount
            } else {
                x1 = (int) Math.round(Utilities.returnX3(xp, yp, xq, yq, w));
                y1 = (int) Math.round(Utilities.returnY3(xp, yp, xq, yq, w));
                x2 = (int) Math.round(Utilities.returnX4(xp, yp, xq, yq, w));
                y2 = (int) Math.round(Utilities.returnY4(xp, yp, xq, yq, w)); //(v.getStrip().getStripIndex())*stripPixelCount
            }
            int width = (int) Math.round(this.width * pixelPerMeter);
            //finds the coordinates of perpendicularly opposite lower(right) points
            if (reverseSegment) {
                x3 = (int) Math.round(Utilities.returnX5(xp, yp, xq, yq, w + width));
                y3 = (int) Math.round(Utilities.returnY5(xp, yp, xq, yq, w + width));
                x4 = (int) Math.round(Utilities.returnX6(xp, yp, xq, yq, w + width));
                y4 = (int) Math.round(Utilities.returnY6(xp, yp, xq, yq, w + width));
            } else {
                x3 = (int) Math.round(Utilities.returnX3(xp, yp, xq, yq, w + width));
                y3 = (int) Math.round(Utilities.returnY3(xp, yp, xq, yq, w + width));
                x4 = (int) Math.round(Utilities.returnX4(xp, yp, xq, yq, w + width));
                y4 = (int) Math.round(Utilities.returnY4(xp, yp, xq, yq, w + width));
            }
            xs = new int[]{x1, x2, x4, x3};
            ys = new int[]{y1, y2, y4, y3};

        }
        g.setColor(color);
        g.fillPolygon(xs, ys, 4);
        vehicleCorners[0] = new Point2D(xs[0], ys[0]);
        vehicleCorners[1] = new Point2D(xs[1], ys[1]);
        vehicleCorners[2] = new Point2D(xs[2], ys[2]);
        vehicleCorners[3] = new Point2D(xs[3], ys[3]);
        if (Parameters.DEBUG_MODE) {
            Font font = new Font("Serif", Font.PLAIN, 64);
            g.setFont(font);
            g.drawString(Integer.toString(vehicleId), x1, y1);
        }
        try {
            traceWriter.write(x1 + " " + x2 + " " + x3 + " " + x4 + " " + y1 + " " + y2 + " " + y3 + " " + y4 + " ");
            traceWriter.write(color.getRed() + " " + color.getBlue() + " " + color.getGreen());
            traceWriter.newLine();
        } catch (IOException ex) {
            Logger.getLogger(Vehicle.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

}
