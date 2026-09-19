package thesisfinal;

import org.apache.commons.math3.distribution.GammaDistribution;
import org.apache.commons.math3.random.Well19937c;

import java.awt.*;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.io.*;
import java.util.ArrayList;
import java.util.Random;
import java.util.StringTokenizer;
import java.util.logging.Level;
import java.util.logging.Logger;

import static java.lang.Integer.min;
import static java.lang.Math.*;
import static thesisfinal.Constants.PEDESTRIANS_ALONG_THE_ROAD_TYPE;
import static thesisfinal.Parameters.random;
import static thesisfinal.Parameters.seed;

class Utilities {
    static GammaDistribution gb = new GammaDistribution(new Well19937c(seed), 1.68256, 0.04169);

    static double returnX3(double x1, double y1, double x2, double y2, double d) {
        double u = x2 - x1;
        double v = y2 - y1;
        double temp = u;
        u = -v;
        v = temp;
        double denom = sqrt(u * u + v * v);
        u = u / denom;
        v = v / denom;
        return x1 + d * u;
    }

    static double returnY3(double x1, double y1, double x2, double y2, double d) {
        double u = x2 - x1;
        double v = y2 - y1;
        double temp = u;
        u = -v;
        v = temp;
        double denom = sqrt(u * u + v * v);
        u = u / denom;
        v = v / denom;
        return y1 + d * v;
    }

    static double returnX4(double x1, double y1, double x2, double y2, double d) {
        double u = x2 - x1;
        double v = y2 - y1;
        double temp = u;
        u = -v;
        v = temp;
        double denom = sqrt(u * u + v * v);
        u = u / denom;
        v = v / denom;
        return x2 + d * u;
    }

    static double returnY4(double x1, double y1, double x2, double y2, double d) {
        double u = x2 - x1;
        double v = y2 - y1;
        double temp = u;
        u = -v;
        v = temp;
        double denom = sqrt(u * u + v * v);
        u = u / denom;
        v = v / denom;
        return y2 + d * v;
    }

    static double returnX5(double x1, double y1, double x2, double y2, double d) {
        double u = x2 - x1;
        double v = y2 - y1;
        double temp = u;
        u = v;
        v = -temp;
        double denom = sqrt(u * u + v * v);
        u = u / denom;
        v = v / denom;
        return x1 + d * u;
    }

    static double returnY5(double x1, double y1, double x2, double y2, double d) {
        double u = x2 - x1;
        double v = y2 - y1;
        double temp = u;
        u = v;
        v = -temp;
        double denom = sqrt(u * u + v * v);
        u = u / denom;
        v = v / denom;
        return y1 + d * v;
    }

    static double returnX6(double x1, double y1, double x2, double y2, double d) {
        double u = x2 - x1;
        double v = y2 - y1;
        double temp = u;
        u = v;
        v = -temp;
        double denom = sqrt(u * u + v * v);
        u = u / denom;
        v = v / denom;
        return x2 + d * u;
    }

    static double returnY6(double x1, double y1, double x2, double y2, double d) {
        double u = x2 - x1;
        double v = y2 - y1;
        double temp = u;
        u = v;
        v = -temp;
        double denom = sqrt(u * u + v * v);
        u = u / denom;
        v = v / denom;
        return y2 + d * v;
    }

    static double getDistance(double x1, double y1, double x2, double y2) {
        return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    }

    public static double gaussianPDF(double x, double mean, double stdDev) {
        return (1 / (stdDev * Math.sqrt(2 * Math.PI))) *
               Math.exp(-0.5 * Math.pow((x - mean) / stdDev, 2));
    }

    // Method to calculate the cumulative distribution function (CDF) for a Gaussian distribution
    public static double cumulativeDistributionFunction(double x, double mean, double stdDev) {
        return 0.5 * (1 + erf((x - mean) / (stdDev * Math.sqrt(2))));
    }

    // Method to calculate the error function (erf) used in the CDF calculation
    public static double erf(double z) {
        // Using numerical approximation for erf(z)
        double t = 1.0 / (1.0 + 0.5 * Math.abs(z));
        double tau = t * Math.exp(-z*z - 1.26551223 + t * (1.00002368 +
                     t * (0.37409196 + t * (0.09678418 + t * (-0.18628806 +
                     t * (0.27886807 + t * (-1.13520398 + t * (1.48851587 +
                     t * (-0.82215223 + t * 0.17087277)))))))));
        return z >= 0 ? 1 - tau : tau - 1;
    }

    static boolean doIntersect(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        Point2D.Double temp1 = new Point2D.Double(x1, y1);
        Point2D.Double temp2 = new Point2D.Double(x2, y2);
        Point2D.Double temp3 = new Point2D.Double(x3, y3);
        Point2D.Double temp4 = new Point2D.Double(x4, y4);
        boolean intersects = Line2D.linesIntersect(temp1.x, temp1.y, temp2.x, temp2.y, temp3.x, temp3.y, temp4.x, temp4.y);
        boolean shareAnyPoint = shareAnyPoint(temp1, temp2, temp3, temp4);
        return intersects && !shareAnyPoint;
    }

    private static boolean shareAnyPoint(Point2D.Double A, Point2D.Double B, Point2D.Double C, Point2D.Double D) {
        if (isPointOnTheLine(A, B, C)) {
            return true;
        } else if (isPointOnTheLine(A, B, D)) {
            return true;
        } else if (isPointOnTheLine(C, D, A)) {
            return true;
        } else {
            return isPointOnTheLine(C, D, B);
        }
    }

    private static boolean isPointOnTheLine(Point2D.Double A, Point2D.Double B, Point2D.Double P) {
        double m = (B.y - A.y) / (B.x - A.x);
        if (Double.isInfinite(m)) {
            return abs(A.x - P.x) < 0.001;
        }
        return abs((P.y - A.y) - m * (P.x - A.x)) < 0.001;
    }

    static double getWeight(Vehicle leader, Vehicle follower, int sideStripsToConsider) {
        int leader_start_strip = leader.getStripIndex();
        int leader_number_strips = leader.getNumberOfStrips();
        int leader_end_strip = leader_start_strip + leader_number_strips - 1;

        int follower_start_strip = follower.getStartStripForMyModel(sideStripsToConsider);
        int follower_end_strip = follower.getEndStripForMyModel(sideStripsToConsider);
        int follower_number_strips = follower_end_strip - follower_start_strip + 1;

        int start_overlap = max(leader_start_strip, follower_start_strip);
        int end_overlap = min(leader_end_strip, follower_end_strip);
        int total_overlap = end_overlap - start_overlap + 1;

        double temp1 = (start_overlap + end_overlap) / 2.0;
        double temp2 = (temp1 - follower_start_strip + 0.5);
        double height = temp2 / follower_number_strips;

        double normalize_height = Math.min(height, 1 - height);

        //TODO Why 0.2???
        // double weight = total_overlap > 0 ? total_overlap * normalize_height : 0.2;

        double weight =  total_overlap * normalize_height;

        return weight;

    }

    static double getWeightByType(Vehicle leader) {
        if (leader.getType() == 12) {
            return Parameters.PEDESTRIAN_WEIGHT;
        }
        return 1;
    }

    /*
        cycle, rickshaw, van, bike, car, car, car, CNG, bus, bus, truck, truck
     */
    static double getCarWidth(int type) {
        double[] carWidths = {0.55, 1.20, 1.22, 0.6, 1.7, 1.76, 1.78, 1.4, 2.4, 2.30, 2.44, 2.46, 0.4};
        int index = abs(type) % min(Constants.TYPES_OF_CARS, carWidths.length);
        return carWidths[index];
    }

    static double getCarLength(int type) {
        double[] carLengths = {1.9, 2.4, 2.5, 1.8, 5, 4.55, 4.3, 2.65, 10.3, 9.5, 7.2, 7.5, 0.4};
        int index = abs(type) % min(Constants.TYPES_OF_CARS, carLengths.length);
        return carLengths[index];
    }


    /**
     * @param type of the car
     * @return the maximum speed of the car in m/s
     */
    static double getCarMaxSpeed(int type) {
        double[] carSpeeds = {15, 8, 7, 100, 100, 60, 110, 40, 30, 40, 35, 45, 5.1}; // km/hour
        int index = abs(type) % min(Constants.TYPES_OF_CARS, carSpeeds.length);
        double speed = carSpeeds[index] * 1000 / 3600; // m/s
        if (type == PEDESTRIANS_ALONG_THE_ROAD_TYPE) {
            double nd = random.nextGaussian();
            return precision2(speed + nd * speed / 5.0);
        } else {
            return precision2(speed);
        }
    }

    /**
     * @param type of the car
     * @return the acceleration in m/s^2
     */
    static double getCarAcceleration(int type) {
        // m/s^2;  approximately maxSpeed/10 i.e. takes 10 seconds to reach max speed
        double[] carAccelerations = {0.42, 0.25, 0.20, 2.80, 3, 2.80, 3.34, 1.11, 0.84, 1.12, 0.96, 1.24, 0.1};
        int index = abs(type) % min(Constants.TYPES_OF_CARS, carAccelerations.length);

        if (type == PEDESTRIANS_ALONG_THE_ROAD_TYPE) {
            double nd = random.nextGaussian();
            return precision2(carAccelerations[index] + nd * carAccelerations[index] / 5.0); // m/s^2
        } else {
            return carAccelerations[index];
        }
    }

    static Parameters.DLC_MODEL getDLCModel(int type) {
        if (type == PEDESTRIANS_ALONG_THE_ROAD_TYPE) {
            return Parameters.DLC_MODEL.NAIVE_MODEL;
//            return Parameters.lane_changing_model;
        } else {
            return Parameters.lane_changing_model;
        }
    }

    static int numberOfStrips(int type) {
        return (int) ceil(getCarWidth(type) / Parameters.stripWidth);
    }
    

    // functions of roadside objects or side friction elements

    static int pedestrianBlockageStrip(double distanceFromFootpath)
    {
        return (int) ceil(distanceFromFootpath / Parameters.stripWidth);
    }

    private static double getProbabilityDensityOfObjectBlockage (NormalDistribution[] distributions, double x) {
		double y = 0;
		for (NormalDistribution distribution : distributions) {
			y += distribution.getProbabilityDensityWithFactor(x);
		}
		return y;
	}

    protected static double getRandomFromMultipleGaussianDistributionOfObjectsBlockage (int objectType) {
		double randomMultiplier = 0;
		NormalDistribution[] distributionsOfObject;

		double startIndex, stopIndex;
		switch (objectType) {
			case 1:
				distributionsOfObject = Constants.STANDING_PEDESTRIAN_DISTRIBUTIONS;
				startIndex = Constants.STANDING_PEDESTRIAN_MIN_BLOCKAGE;
				stopIndex = Constants.STANDING_PEDESTRIAN_MAX_BLOCKAGE;
				break;
			case 2:
				distributionsOfObject = Constants.PARKED_CAR_DISTRIBUTIONS;
				startIndex = Constants.PARKED_CAR_MIN_BLOCKAGE;
				stopIndex = Constants.PARKED_CAR_MAX_BLOCKAGE;
				break;
			case 3:
				distributionsOfObject = Constants.PARKED_RICKSHAW_DISTRIBUTIONS;
				startIndex = Constants.PARKED_RICKSHAW_MIN_BLOCKAGE;
				stopIndex = Constants.PARKED_RICKSHAW_MAX_BLOCKAGE;
				break;
			case 4:
				distributionsOfObject = Constants.PARKED_CNG_DISTRIBUTIONS;
				startIndex = Constants.PARKED_CNG_MIN_BLOCKAGE;
				stopIndex = Constants.PARKED_CNG_MAX_BLOCKAGE;
				break;
            case 5:
                distributionsOfObject = Constants.WALKING_PEDESTRIAN_DISTRIBUTIONS;
                startIndex = Constants.WALKING_PEDESTRIAN_MIN_BLOCKAGE;
                stopIndex = Constants.WALKING_PEDESTRIAN_MAX_BLOCKAGE;
                break;
			default:
				System.out.println("Error: Distribution for objectType=" + objectType + " does not exist.");
				return 0; //Distance from footpath will be zero.
		}

		for (double i = startIndex; i<= stopIndex; i+=0.01) {
			randomMultiplier += getProbabilityDensityOfObjectBlockage(distributionsOfObject, i);
		}
		Random r = new Random();
		double randomDouble = r.nextDouble() * randomMultiplier;

		//For each possible integer return value, subtract yourFunction value for that possible return value till you get below 0.  Once you get below 0, return the current value.
		double theRandomNumber = startIndex;
		randomDouble = randomDouble - getProbabilityDensityOfObjectBlockage(distributionsOfObject, theRandomNumber);
		while (randomDouble >= 0) {
			theRandomNumber += 0.01;
			randomDouble = randomDouble - getProbabilityDensityOfObjectBlockage(distributionsOfObject, theRandomNumber);
		}

		return theRandomNumber;
	}
    
    public static int randInt(int min, int max) {
        // nextInt is normally exclusive of the top value,
        // so add 1 to make it inclusive
        int randomNum = random.nextInt((max - min) + 1) + min;

        return randomNum;
    }

    @SuppressWarnings("Duplicates")
    static thesisfinal.Point2D getNewEndPointForIntersectionStrip(Vehicle vehicle, IntersectionStrip is, int newStripIndex) {
        thesisfinal.Point2D endPoint;
        Segment enteringSegment = is.enteringSegment;
        double w1 = 1 * Parameters.pixelPerFootpathStrip + (newStripIndex - 1) * Parameters.pixelPerStrip;
        double x_1 = enteringSegment.getStartX() * Parameters.pixelPerMeter;
        double y_1 = enteringSegment.getStartY() * Parameters.pixelPerMeter;
        double x_2 = enteringSegment.getEndX() * Parameters.pixelPerMeter;
        double y_2 = enteringSegment.getEndY() * Parameters.pixelPerMeter;

        double x_3 = returnX3(x_1, y_1, x_2, y_2, w1);
        double y_3 = returnY3(x_1, y_1, x_2, y_2, w1);
        double x_4 = returnX4(x_1, y_1, x_2, y_2, w1);
        double y_4 = returnY4(x_1, y_1, x_2, y_2, w1);
        double dist1 = (x_3 - vehicle.getSegEndX() * Parameters.pixelPerMeter) * (x_3 - vehicle.getSegEndX() * Parameters.pixelPerMeter) + (y_3 - vehicle.getSegEndY() * Parameters.pixelPerMeter) * (y_3 - vehicle.getSegEndY() * Parameters.pixelPerMeter);
        double dist2 = (x_4 - vehicle.getSegEndX() * Parameters.pixelPerMeter) * (x_4 - vehicle.getSegEndX() * Parameters.pixelPerMeter) + (y_4 - vehicle.getSegEndY() * Parameters.pixelPerMeter) * (y_4 - vehicle.getSegEndY() * Parameters.pixelPerMeter);
        if (dist1 < dist2) {    // choosing closest segment end point from vehicle
            endPoint = new thesisfinal.Point2D(x_3, y_3);
        } else {
            endPoint = new thesisfinal.Point2D(x_4, y_4);
        }
        return endPoint;
    }

    static void drawTrace(LineNumberReader traceReader, Graphics g) {
        try {
            String s;
            do {
                s = traceReader.readLine();
            } while (s != null && !s.startsWith("Simulation"));
            /*
             here first line Current pedestrians
             then the pedestrians list
             then 1 line Current vehicles
             then the vehicles list
            */
            if (s != null && s.startsWith("Simulation")) {
                StringTokenizer tokenizer = new StringTokenizer(s);
                tokenizer.nextToken();
                int simStep = Integer.parseInt(tokenizer.nextToken());
                Parameters.showProgressSlider.setValue(simStep);
            }
            traceReader.readLine();

            if (Parameters.acrossPedestrianMode) {
                while (true) {
                    s = traceReader.readLine();
                    if (s == null || s.startsWith("Current")) {
                        break;
                    }
                    StringTokenizer tokenizer = new StringTokenizer(s);
                    int x, y;
                    boolean inAccident;
                    x = Integer.parseInt(tokenizer.nextToken());
                    y = Integer.parseInt(tokenizer.nextToken());
                    inAccident = Boolean.parseBoolean(tokenizer.nextToken());
                    if (inAccident) {
                        g.setColor(Color.red);
                        g.fillOval(x, y, 10, 10);
                    } else {
                        g.setColor(Constants.pedestrianColor);
                        g.fillOval(x, y, 7, 7);
                    }
                }
            }
            // Lines after current vehicles
            while (true) {
                s = traceReader.readLine();
                if (s == null || s.startsWith("End")) {
                    break;
                }
                StringTokenizer tokenizer = new StringTokenizer(s);
                int[] xs = new int[4];
                int[] ys = new int[4];
                int red, green, blue;
                xs[0] = Integer.parseInt(tokenizer.nextToken());
                xs[1] = Integer.parseInt(tokenizer.nextToken());
                xs[3] = Integer.parseInt(tokenizer.nextToken());
                xs[2] = Integer.parseInt(tokenizer.nextToken());

                ys[0] = Integer.parseInt(tokenizer.nextToken());
                ys[1] = Integer.parseInt(tokenizer.nextToken());
                ys[3] = Integer.parseInt(tokenizer.nextToken());
                ys[2] = Integer.parseInt(tokenizer.nextToken());

                red = Integer.parseInt(tokenizer.nextToken());
                green = Integer.parseInt(tokenizer.nextToken());
                blue = Integer.parseInt(tokenizer.nextToken());
                Color c = new Color(red, green, blue);

                g.setColor(c);
                g.fillPolygon(xs, ys, 4);
            }
        } catch (IOException e) {
            e.printStackTrace();
        }

    }

    static double precision2(double d) {
        return ((int) (d * 10000)) / 10000.0;
    }

    static double truncatedGaussian(double sigma, double lb, double ub) {
        double x = 0;
        for (int i = 0; i < 5; i++) {
            x = random.nextGaussian() * sigma;
            if (lb <= x && x <= ub) {
                return x;
            }
        }
        if (x <= lb) {
            return lb;
        } else if (x >= ub) {
            return ub;
        } else {
            assert false;
            return 0;
        }
    }

    /**
     * @return a time penalty for collision in minutes following gamma
     * random variable
     */
    static double getCollisionPenalty() {
        return gb.sample();
    }

    static void initialize() {
        try {
            BufferedReader bufferedReader = new BufferedReader(new FileReader(new File("input/parameter.txt")));
            while (true) {
                String dataLine = bufferedReader.readLine();
                if (dataLine == null) {
                    bufferedReader.close();
                    break;
                }
                StringTokenizer stringTokenizer = new StringTokenizer(dataLine);
                String name = stringTokenizer.nextToken();
                String value = stringTokenizer.nextToken();
                switch (name) {
                    case "SimulationEndTime":
                        Parameters.simulationEndTime = Integer.parseInt(value);
                        break;
                    case "PixelPerMeter":
                        Parameters.pixelPerMeter = Integer.parseInt(value);
                        break;
                    case "SimulationSpeed":
                        Parameters.simulationSpeed = Integer.parseInt(value);
                        break;
                    case "EncounterPerAccident":
                        Parameters.encounterPerAccident = Double.parseDouble(value);
                        break;
                    case "StripWidth":
                        Parameters.stripWidth = Double.parseDouble(value);
                        break;
                    case "FootpathStripWidth":
                        Parameters.footpathStripWidth = Double.parseDouble(value);
                        break;
                    case "MaximumSpeed":
                        Parameters.maximumSpeed = Double.parseDouble(value);
                        break;
                    case "AcrossPedestrianMode":
                        Parameters.acrossPedestrianMode = value.equalsIgnoreCase("On");
                        break;
                    case "AlongPedestrianMode":
                        Parameters.alongPedestrianMode = value.equalsIgnoreCase("On");
                        break;
                    case "DebugMode":
                        Parameters.DEBUG_MODE = value.equalsIgnoreCase("On");
                        break;
                    case "ObjectMode":
                        Parameters.OBJECT_MODE = value.equalsIgnoreCase("On");
                        break;
                    case "TraceMode":
                        Parameters.TRACE_MODE = value.equalsIgnoreCase("On");
                        break;
                    case "RandomSeed":
                        seed = new Random().nextInt(101);
                        break;
                    case "SignalChangeDuration":
                        Parameters.SIGNAL_CHANGE_DURATION = Integer.parseInt(value);
                        Parameters.SIGNAL_CHANGE_DURATION = (int)(round(Parameters.SIGNAL_CHANGE_DURATION / Constants.TIME_STEP));
                        break;
                    case "DefaultTranslateX":
                        Parameters.DEFAULT_TRANSLATE_X = Double.parseDouble(value);
                        break;
                    case "DefaultTranslateY":
                        Parameters.DEFAULT_TRANSLATE_Y = Double.parseDouble(value);
                        break;
                    case "CenteredView":
                        Parameters.CENTERED_VIEW = value.equalsIgnoreCase("on");
                        break;
                    case "DLC_model":
                        int o = Integer.parseInt(value);
                        switch (o) {
                            case 0 -> Parameters.lane_changing_model = Parameters.DLC_MODEL.NAIVE_MODEL;
                            case 1 -> Parameters.lane_changing_model = Parameters.DLC_MODEL.GHR_MODEL;
                            case 2 -> Parameters.lane_changing_model = Parameters.DLC_MODEL.GIPPS_MODEL;
                            case 3 -> Parameters.lane_changing_model = Parameters.DLC_MODEL.MOBIL_MODEL;
                            default -> Parameters.lane_changing_model = Parameters.DLC_MODEL.GIPPS_MODEL;
                        }
                        break;
                    case "CF_model":
                        int p = Integer.parseInt(value);
                        switch (p) {
                            case 0 -> Parameters.car_following_model = Parameters.CAR_FOLLOWING_MODEL.HYBRID_MODEL;
                            case 1 -> Parameters.car_following_model = Parameters.CAR_FOLLOWING_MODEL.NAIVE_MODEL;
                            case 2 -> Parameters.car_following_model = Parameters.CAR_FOLLOWING_MODEL.GIPPS_MODEL; // no collision
                            case 3 -> Parameters.car_following_model = Parameters.CAR_FOLLOWING_MODEL.KRAUSS_MODEL; //100700+ //OK
                            case 4 -> Parameters.car_following_model = Parameters.CAR_FOLLOWING_MODEL.GFM_MODEL; // no collision
                            case 5 -> Parameters.car_following_model = Parameters.CAR_FOLLOWING_MODEL.IDM_MODEL; // 61021  //OK
                            case 6 -> Parameters.car_following_model = Parameters.CAR_FOLLOWING_MODEL.RVF_MODEL; //8284
                            case 7 -> Parameters.car_following_model = Parameters.CAR_FOLLOWING_MODEL.VFIAC_MODEL; //4654
                            case 8 -> Parameters.car_following_model = Parameters.CAR_FOLLOWING_MODEL.OVCM_MODEL; // no collision (1s->time step) 15577
                            case 9 -> Parameters.car_following_model = Parameters.CAR_FOLLOWING_MODEL.KFTM_MODEL; // 12
                            case 10 -> Parameters.car_following_model = Parameters.CAR_FOLLOWING_MODEL.HDM_MODEL; // 22742
                            case 11 -> Parameters.car_following_model = Parameters.CAR_FOLLOWING_MODEL.SBM_MODEL; // 8363
                            case 12 -> Parameters.car_following_model = Parameters.CAR_FOLLOWING_MODEL.MY_MODEL; // no collision
                            default -> Parameters.car_following_model = Parameters.CAR_FOLLOWING_MODEL.HYBRID_MODEL;
                        }
                        break;
                    case "SlowVehicle":
                        Parameters.slowVehiclePercentage = Double.parseDouble(value);
                        break;
                    case "MediumVehicle":
                        Parameters.mediumVehiclePercentage = Double.parseDouble(value);
                        break;
                    case "FastVehicle":
                        Parameters.fastVehiclePercentage = Double.parseDouble(value);
                        break;
                    case "TTC_Threshold":
                        Parameters.TTC_THRESHOLD = Double.parseDouble(value);
                        break;
                    case "VehicleGenerationRate":
                        int q = Integer.parseInt(value);
                        switch (q) {
                            case 0 -> Parameters.vehicle_generation_rate = Parameters.VEHICLE_GENERATION_RATE.CONSTANT;
                            case 1 -> Parameters.vehicle_generation_rate = Parameters.VEHICLE_GENERATION_RATE.POISSON;
                            default -> Parameters.vehicle_generation_rate = Parameters.VEHICLE_GENERATION_RATE.CONSTANT;
                        }
                        break;
                    case "ErrorMode":
                        Parameters.ERROR_MODE = value.equalsIgnoreCase("On");
                        break;
                    case "FTMethod":
                        Parameters.FT_METHOD = Integer.parseInt(value);
                        break;
                    case "NoOfReadings":
                        Parameters.NO_OF_READINGS = Integer.parseInt(value);
                        break;
                    case "MFactor":
                        Parameters.M_FACTOR = Double.parseDouble(value);
                        break;
                    case "GUIMode":
                        Parameters.GUI_MODE = value.equalsIgnoreCase("on");
                        break;
                    case "ALPHA":
                        Parameters.ALPHA = Double.parseDouble(value);
                        break;
                    case "BETA":
                        Parameters.BETA = Double.parseDouble(value);
                        break;
                    case "ETA":
                        Parameters.ETA = Double.parseDouble(value);
                        break;
                    case "AcrossPedestrianLimit":
                        Parameters.ACROSS_PEDESTRIAN_LIMIT = Integer.parseInt(value);
                        break;
                    case "AcrossPedestrianPerHour":
                        Parameters.ACROSS_PEDESTRIAN_PER_HOUR = Double.parseDouble(value);
                        break;
                    case "AlongPedestrianPerHour":
                        Parameters.ALONG_PEDESTRIAN_PER_HOUR = Double.parseDouble(value);
                        break;
                    case "AlongPedestrianPercentage":
                        Parameters.ALONG_PEDESTRIAN_PERCENTAGE = Integer.parseInt(value);
                        break;
                    case "DensityPercentage":
                        Parameters.DENSITY_PERCENTAGE = Integer.parseInt(value);
                        break;
                    case "PedestrianWeight":
                        Parameters.PEDESTRIAN_WEIGHT = Double.parseDouble(value);
                        break;
                    case "PedestrianRandomLaneChangePercentage":
                        Parameters.PEDESTRIAN_RANDOM_LANE_CHANGE_PERCENTAGE = Integer.parseInt(value);
                        break;
                    case "PedestrianLeftBiasPercentage":
                        Parameters.PEDESTRIAN_LEFT_BIAS_PERCENTAGE = Integer.parseInt(value);
                        break;
                    case "PenaltyWait":
                        Parameters.PENALTY_WAIT = value.equalsIgnoreCase("on");
                        break;
                    case "ConsiderMinimum":
                        Parameters.CONSIDER_MINIMUM = value.equalsIgnoreCase("on");
                        break;
                    case "NoOfRoutes":
                        Parameters.NO_OF_ROUTES_FOR_STAT = Integer.parseInt(value);
                        break;
                    case "BrakeHard":
                        Parameters.BRAKE_HARD = value.equalsIgnoreCase("on");
                        break;
                    case "AttackScenario":
                        Parameters.ATTACK_SCENARIO = Integer.parseInt(value);
                        break;
                    case "DelayProbability":
                        Parameters.DELAY_PROBABILITY = Double.parseDouble(value);
                        break;
                    case "JitterMagnitude":
                        Parameters.JITTER_MAGNITUDE = Double.parseDouble(value);
                        break;
                    case "DelayMean":
                        Parameters.DELAY_MEAN = Double.parseDouble(value);
                        break;
                    case "DelaySpread":
                        Parameters.DELAY_SPREAD = Double.parseDouble(value);
                        break;
                    case "PacketLossRate":
                        Parameters.PACKET_LOSS_RATE = Double.parseDouble(value);
                        break;
                    case "AttackStripMod":
                        Parameters.ATTACK_STRIP_MOD = Integer.parseInt(value);
                        break;
                    case "AttackStartTime":
                        Parameters.ATTACK_START_TIME = Integer.parseInt(value);
                        break;
                    case "AttackEndTime":
                        Parameters.ATTACK_END_TIME = Integer.parseInt(value);
                        break;
                    case "JitterOnlyMagnitude":
                        Parameters.JITTER_ONLY_MAGNITUDE = Integer.parseInt(value);
                        break;
                    case "SingleShotDelay":
                        Parameters.SINGLE_SHOT_DELAY = Double.parseDouble(value);
                        break;
                    case "SingleShotTime":
                        Parameters.SINGLE_SHOT_TIME = Integer.parseInt(value);
                        break;
                    case "PositionBufferSize":
                        Parameters.POSITION_BUFFER_SIZE = Integer.parseInt(value);
                        break;
                    case "AcrossPedestrianPercentage":
                        System.out.println("Should change the parameter name: AcrossPedestrianPerHour");
                        assert false;
                        break;
                    default:
                        break;
                }
            }
            double value = Parameters.encounterPerAccident;
            if (value < 1) {
                value = 100 / value;
            } else {
                value = 100 - value + 1;
            }
            Parameters.encounterPerAccident = value;
            Parameters.pixelPerFootpathStrip = Parameters.pixelPerMeter * Parameters.footpathStripWidth;
            Parameters.pixelPerStrip = Parameters.pixelPerMeter * Parameters.stripWidth;
            random = seed < 0 ? new Random() : new Random(seed);
            assert (round(Parameters.slowVehiclePercentage + Parameters.mediumVehiclePercentage + Parameters.fastVehiclePercentage) == 100.0);
            // System.out.println("hello"+Parameters.slowVehiclePercentage + " " + Parameters.mediumVehiclePercentage + " " + Parameters.fastVehiclePercentage);
            // Values already parsed from config above; do not override

        } catch (IOException ex) {
            Logger.getLogger(DhakaSimPanel.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    static void indexTraceFile() {
        if (Parameters.TRACE_MODE) {
            Parameters.simulationStepLineNos  = new ArrayList<>(Parameters.simulationEndTime);
            try(LineNumberReader traceReader = new LineNumberReader(new FileReader("trace.txt"))) {
                String s;
                while((s = traceReader.readLine()) != null) {
                    if (s.startsWith("Sim")) {
                        Parameters.simulationStepLineNos.add(traceReader.getLineNumber());
                    }
                }
            } catch (IOException ex) {
                ex.printStackTrace();
            }
        }
    }

}
