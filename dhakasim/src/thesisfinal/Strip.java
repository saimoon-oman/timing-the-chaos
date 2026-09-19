package thesisfinal;

import static thesisfinal.Constants.THRESHOLD_DISTANCE;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Random;

public class Strip {

    private final int segmentIndex;
    private final int stripIndex;
    private final boolean isFootPathStrip;
    private static final Random rand = Parameters.random;

    private final int parentLinkId;

    // we maintain lists for vehicles, pedestrians and roadside objects present in the strip

    private final LinkedList<Vehicle> vehicleList = new LinkedList<>();
    private final LinkedList<Pedestrian> pedestrianList = new LinkedList<>();
    private ArrayList<Object> objectList = new ArrayList<>();

    //Constructor sets segment index and strip index

    public Strip(int segIndex, int strIndex, boolean isFootPathStrip, int parentLinkId) {
        segmentIndex = segIndex;
        stripIndex = strIndex;
        this.isFootPathStrip = isFootPathStrip;
        this.parentLinkId = parentLinkId;
    }


    public boolean isFp() {
        return isFootPathStrip;
    }

    //gets strip index
    public int getStripIndex() {
        return stripIndex;
    }

    //adds vehicle to the strip's vehicle list when vehicle comes over the strip
    void addVehicle(Vehicle v) {
        if (parentLinkId != v.getLink().getId()) {
            System.out.println("Very big problem");
        }
        vehicleList.add(v);
    }

    //removes vehicle from the strip's vehicle list when vehicle lefts the strip
    void delVehicle(Vehicle v) {
        vehicleList.remove(v);
    }

    void addPedestrian(Pedestrian p) {
        pedestrianList.add(p);
    }

    void delPedestrian(Pedestrian p) {
        pedestrianList.remove(p);
    }

    // functions related to roadside objects
    
    void addObject(Object object) {
		objectList.add(object);
	}

    void delObject(Object object) {
		objectList.remove(object);
	}

    boolean hasGapForObject(Object p) {
		double lowerLimit, upperLimit, thresholdDistance = 0.08;
		Vehicle v;

		boolean accident = (rand.nextInt() % (int) Parameters.encounterPerAccident == 0);
		for (Vehicle vehicle : vehicleList) {
			v = vehicle;
			upperLimit = v.getDistanceInSegment() + v.getLength() + thresholdDistance;
			lowerLimit = v.getDistanceInSegment();

			if (v.isReverseSegment()) {
				double lower = lowerLimit;
				double upper = upperLimit;
				lowerLimit = v.getLink().getSegment(v.getSegmentIndex()).getLength() - upper;
				upperLimit = v.getLink().getSegment(v.getSegmentIndex()).getLength() - lower;
			}

			if (lowerLimit < p.getInitPos() && p.getInitPos() < upperLimit) {
				if (!accident) {
					return false;
				}

				p.getSegment().setAccidentCount(p.getSegment().getAccidentCount() + 1); //updateAccidentcount();
				//System.out.println("Type 2: পথচারী নিজেই ধাক্কা লাগাইসে");
				p.inAccident = true;
				delObject(p);
			}
		}
		return true;
	}

    boolean hasGapForMoveAlongPositive(Pedestrian p) {
        double distInSegment = p.getDistanceInSegment();

        double newDistInSegment = distInSegment + p.getSpeed();

        for (Vehicle vehicle : vehicleList) {
            double vStart = vehicle.getDistanceInSegment();
            double vEnd = vStart + vehicle.getLength() + vehicle.getThresholdDistance();
            if (vStart < newDistInSegment && newDistInSegment < vEnd) {
                return false;
            }
        }

        // checking for objects also 
        for (Object object : objectList) {
            double vStart = object.getDistanceInSegment();
            double vEnd = vStart + object.getObjectLength() + THRESHOLD_DISTANCE;
            if (vStart < newDistInSegment && newDistInSegment < vEnd) {
                return false;
            }
        }

        return true;
    }

    boolean hasGapForMoveAlongNegative(Pedestrian p) {
        double distInSegment = p.getDistanceInSegment();

        double newDistInSegment = distInSegment - p.getSpeed();

        for (Vehicle vehicle : vehicleList) {
            double vStart = vehicle.getDistanceInSegment();
            double vEnd = vStart + vehicle.getLength() + vehicle.getThresholdDistance();
            if (vStart < newDistInSegment && newDistInSegment < vEnd) {
                return false;
            }
        }

        // checking for objects also 
        for (Object object: objectList) {
            double vStart = object.getDistanceInSegment();
            double vEnd = vStart + object.getObjectLength() + THRESHOLD_DISTANCE;
            if (vStart < newDistInSegment && newDistInSegment < vEnd) {
                return false;
            }
        }

        return true;
    }

    Vehicle getProbableLeaderForMyModel(Vehicle follower) {
        
        Vehicle pedLeader = probableLeaderForPedestrian(follower);
        Vehicle vehLeader = probableLeader(follower);

        if (pedLeader == null)
            return vehLeader;
        if (vehLeader == null)
            return pedLeader;

        return pedLeader.getDistanceInSegment() < vehLeader.getDistanceInSegment() ? pedLeader : vehLeader;
    }

    Vehicle probableLeaderForPedestrian(Vehicle follower) {
        double min = Double.MAX_VALUE;
        Pedestrian ped = null;
        // as accidents can occur so getLength is omitted
        double distance = follower.getDistanceInSegment() + follower.getLength(); // + follower.getLength();
        for (Pedestrian leader : pedestrianList) {
            if (leader.getDistanceInSegment() > distance) {
                double compare = leader.getDistanceInSegment() - distance;
                if (compare < min) {
                    min = compare;
                    ped = leader;
                }
            }
        }
        Vehicle res = null;
        if (ped != null) {
            res = follower.createDummyVehicleAtPedestrianPositionForMyModel(ped);
        }
        return res;
    }

     ArrayList<Integer> getVehiclesInRange(double startingDistance, double endingDistance) {
        ArrayList<Integer> vIds = new ArrayList<>();

        for (Vehicle vehicle : vehicleList) {
            double lowerLimit = vehicle.getDistanceInSegment();
            double upperLimit = vehicle.getDistanceInSegment() + vehicle.getLength();
            if ((startingDistance >= lowerLimit && startingDistance <= upperLimit
                    || endingDistance >= lowerLimit && endingDistance <= upperLimit)
                    || (lowerLimit >= startingDistance && lowerLimit <= endingDistance
                    || endingDistance >= startingDistance && upperLimit <= endingDistance)) {
                vIds.add(vehicle.getVehicleId());
            }
        }
        return vIds;
     }

    ArrayList<Integer> getPedestriansInRange(double startingDistance, double endingDistance) {
        ArrayList<Integer> pIds = new ArrayList<>();

        for (Pedestrian pedestrian : pedestrianList) {
            double distInSegment = pedestrian.getDistanceInSegment();
            if (startingDistance < distInSegment && distInSegment < endingDistance)
                pIds.add(pedestrian.getPedestrianId());
        }
        return pIds;
    }

    boolean hasCollisionOccurred(Vehicle v) {
        double lowerLimit = v.getDistanceInSegment();
        double upperLimit = lowerLimit + v.getLength();

        for (Vehicle vehicle : vehicleList) {
            if (vehicle == v)
                continue;

            if(v.getSegment() != vehicle.getSegment() || v.isReverseSegment() != vehicle.isReverseSegment())
                continue;

            double distance = vehicle.getDistanceInSegment();
            if (lowerLimit < distance && distance < upperLimit) {
                return true;
            }
        }

        return false;
    }

    Vehicle getAccidentVehicle(Vehicle v) {
        double lowerLimit = v.getDistanceInSegment();
        double upperLimit = lowerLimit + v.getLength();

        for (Vehicle vehicle : vehicleList) {
            if (vehicle == v)
                continue;

            double distance = vehicle.getDistanceInSegment();
            if (lowerLimit < distance && distance < upperLimit) {
                return vehicle;
            }
        }

        return null;
    }
    //for a vehicle on this strip,finds another vehicle on the same strip with minimum distance ahead.
    Vehicle probableLeader(Vehicle follower) {
        double min = Double.MAX_VALUE;
        Vehicle ret = null;
        // as accidents can occur so getLength is omitted
        double distance = follower.getDistanceInSegment() + follower.getLength();
        for (Vehicle leader : vehicleList) {
            if (leader.getDistanceInSegment() > distance) {
                double compare = leader.getDistanceInSegment() - distance;
                if (compare < min) {
                    min = compare;
                    ret = leader;
                }
            }
        }
        return ret;
    }

    Object probableObjectLeader(Vehicle follower)
    {
        double min = Double.MAX_VALUE;
        Object ret = null;
        // as accidents can occur so getLength is omitted
        double distance = follower.getDistanceInSegment() + follower.getLength();
        for (Object leader : objectList) {
            if (leader.getDistanceInSegment() > distance) {
                double compare = leader.getDistanceInSegment() - distance;
                if (compare < min) {
                    min = compare;
                    ret = leader;
                }
            }
        }
        return ret;
    }

    Vehicle probableFollower(Vehicle leader) {
        double min = Double.MAX_VALUE;
        Vehicle ret = null;
        for(Vehicle follower : vehicleList) {
            double distance = follower.getDistanceInSegment() + follower.getLength();
            if (leader.getDistanceInSegment() > distance) {
                double compare = leader.getDistanceInSegment() - distance;
                if (compare < min) {
                    min = compare;
                    ret = follower;
                }
            }
        }
        return ret;
    }

    //checks whether there is space for a vehicle to move forward without a collision
    //and keeping a threshold distance
    double getGapForForwardMovement(Vehicle v) {
        double forwardGap;
        double thresholdDistance = v.getThresholdDistance();
        double upperLimit, lowerLimit;

        boolean accident = true;

        Vehicle leader = getProbableLeaderForMyModel(v);
        if (leader == null) {
            forwardGap = v.getLink().getSegment(v.getSegmentIndex()).getLength() - v.getDistanceInSegment() - v.getLength();
        } else {
            forwardGap = Vehicle.getGap(leader, v);
        }

        if (forwardGap <= 0) {
            return 0;
        }

        if (v.isReverseSegment()) {
            lowerLimit = v.getDistanceInSegment() + v.getLength() + v.getSpeed() * Constants.TIME_STEP + thresholdDistance;
            upperLimit = v.getDistanceInSegment() + v.getLength();
        } else {
            upperLimit = v.getDistanceInSegment() + v.getLength() + v.getSpeed() * Constants.TIME_STEP + thresholdDistance;
            lowerLimit = v.getDistanceInSegment() + v.getLength();
        }

        if (v.isReverseSegment()) {
            double lower = lowerLimit;
            double upper = upperLimit;
            lowerLimit = v.getLink().getSegment(v.getSegmentIndex()).getLength() - upper;
            upperLimit = v.getLink().getSegment(v.getSegmentIndex()).getLength() - lower;
        }

//        if (rand.nextInt() % (int) Parameters.encounterPerAccident == 0) {
//            accident = true;
//        }
        ArrayList<Pedestrian> pedestriansToRemove = new ArrayList<>();
        for (Pedestrian pedestrian : pedestrianList) {
            double pedestrianPos = pedestrian.getInitPos();
            if (lowerLimit < pedestrianPos && pedestrianPos < upperLimit) {
//                if (!accident || Parameters.car_following_model == Parameters.CAR_FOLLOWING_MODEL.NAIVE_MODEL) {
//                    forwardGap = Math.min(forwardGap, pedestrianPos - lowerLimit - 0.1); // 0.1 is a threshold
//                } else {
//                    pedestrian.getSegment().setAccidentCount(pedestrian.getSegment().getAccidentCount() + 1);//updateAccidentcount();
//                    pedestrian.inAccident = true;
//                    pedestriansToRemove.add(pedestrian);
//                }
                forwardGap = 0;
            }
        }
        pedestrianList.removeAll(pedestriansToRemove);
        return Math.max(forwardGap, 0);

    }

    boolean checkForAccident(Vehicle v) {
        for (Pedestrian p : pedestrianList) {
            double distInSegment = p.getDistanceInSegment();
            if (distInSegment > v.getDistanceInSegment() && distInSegment < v.getDistanceInSegment() + v.getLength()) {
                System.out.printf("Accident: SimStep: %d Vehicle %d Pedestrian %d\n", Parameters.simulationStep, v.getVehicleId(), p.getPedestrianId());
                p.inAccident = true;
                p.setToRemove(true);
                this.pedestrianList.remove(p);
                return true;
            }
        }
        return false;
    }

    boolean hasGapForPedestrian(Pedestrian p) {
        double lowerLimit, upperLimit, thresholdDistance = 0.08;
        Vehicle v;

//        boolean accident = (rand.nextInt() % (int) Parameters.encounterPerAccident == 0);
//        boolean accident = true;
        for (Vehicle vehicle : vehicleList) {
            v = vehicle;
            upperLimit = v.getDistanceInSegment() + v.getLength() + thresholdDistance;
            lowerLimit = v.getDistanceInSegment();
            //____________________
            if (v.isReverseSegment()) {
                double lower = lowerLimit;
                double upper = upperLimit;
                lowerLimit = v.getLink().getSegment(v.getSegmentIndex()).getLength() - upper;
                upperLimit = v.getLink().getSegment(v.getSegmentIndex()).getLength() - lower;
            }
            //____________________
            if (lowerLimit < p.getInitPos() && p.getInitPos() < upperLimit) {
//                if (!accident || Parameters.car_following_model == Parameters.CAR_FOLLOWING_MODEL.NAIVE_MODEL) {
//                    return false;
//                }  //System.out.println(lowerLimit + " " + obj.getInitPos() + " " + upperLimit);
                return false;

//                p.getSegment().setAccidentCount(p.getSegment().getAccidentCount() + 1);//updateAccidentcount();
//                p.inAccident = true;
//                delPedestrian(p);
            }
        }

        //checking gap with respect to objects

        for (Object object : objectList) {
            
            upperLimit = object.getDistanceInSegment() + object.getObjectLength() + thresholdDistance;
            lowerLimit = object.getDistanceInSegment();           
            if (object.isReverseSegment()) {
                double lower = lowerLimit;
                double upper = upperLimit;
                lowerLimit = object.getSegment().getLength() - upper;
                upperLimit = object.getSegment().getLength() - lower;
            }           
            if (lowerLimit < p.getInitPos() && p.getInitPos() < upperLimit) {
                return false;
            }
        }



        return true;
    }

    //checks whether there is adequate space for adding a new vehicle
    boolean hasGapForAddingVehicle(double vehicleLength) {
        /*
         * new vehicle enters if it has at least THRESHOLD DISTANCE gap after entering
         * with leader vehicle
         */

        double lowerLimit = 0.08;
        double upperLimit = Constants.THRESHOLD_DISTANCE + 0.08 + vehicleLength;
        // boolean accident = (rand.nextInt() % (int) Parameters.encounterPerAccident == 0);
        for (Vehicle vehicle : vehicleList) {
            if ((vehicle.getDistanceInSegment() < upperLimit
                    && vehicle.getDistanceInSegment() > lowerLimit)) {
                return false;
            }

        }

        for(Object object: objectList)
        {
            if((object.getDistanceInSegment() < upperLimit && object.getDistanceInSegment() > lowerLimit))
            {
                return false;
            }
        }

        ArrayList<Pedestrian> pedestriansToRemove = new ArrayList<>();
        for (Pedestrian pedestrian : pedestrianList) {
            double objpos = pedestrian.getInitPos();
            if (lowerLimit < objpos && objpos < upperLimit) {
                return false;//                
            }
        }
        pedestrianList.removeAll(pedestriansToRemove);
        return true;

    }

    boolean hasGapForAddingObject(double objectLength, double initpos) {
        /*
         * new vehicle enters if it has at least THRESHOLD DISTANCE gap after entering
         * with leader vehicle
         */

        double threshold_distance = 0.5;

        // double lowerLimit = 0.08;
        // double upperLimit = Constants.THRESHOLD_DISTANCE + 0.08 + vehicleLength;
        // boolean accident = (rand.nextInt() % (int) Parameters.encounterPerAccident == 0);
        for (Vehicle vehicle : vehicleList) {
            if (vehicle.getDistanceInSegment() + vehicle.getLength() + threshold_distance > initpos
                    && vehicle.getDistanceInSegment() < initpos + objectLength + threshold_distance) {
                return false;
            }

        }

        for(Object object: objectList)
        {
            if(object.getDistanceInSegment() + object.getObjectLength() + threshold_distance > initpos 
                && object.getDistanceInSegment() < initpos + objectLength + threshold_distance){            
                return false;
            }
        }
        
        return true;

    }

    /*similar to isGapForMoveForward but doesn't consider vehicle speed, checks whether there
     *is enough space for a given vehicle forward movement.
     */
    boolean hasGapForStripChange(Vehicle subjectVehicle, Vehicle leader, Vehicle follower) {
        double thresholdDistance = subjectVehicle.getThresholdDistance();
        double lowerLimit1 = subjectVehicle.getDistanceInSegment() - thresholdDistance;
        double upperLimit1 = subjectVehicle.getDistanceInSegment() + subjectVehicle.getLength() + thresholdDistance;

        for (Vehicle vehicle : vehicleList) {
            if (subjectVehicle == vehicle) {
                continue;
            }
            double lowerLimit2 = vehicle.getDistanceInSegment() - thresholdDistance;
            double upperLimit2 = vehicle.getDistanceInSegment() + vehicle.getLength() + thresholdDistance;
            if ((lowerLimit1 >= lowerLimit2 && lowerLimit1 <= upperLimit2
                    || upperLimit1 >= lowerLimit2 && upperLimit1 <= upperLimit2)
                    || (lowerLimit2 >= lowerLimit1 && lowerLimit2 <= upperLimit1
                    || upperLimit2 >= lowerLimit1 && upperLimit2 <= upperLimit1)) {
                return false;
            }
        }
        //____________________
        if (subjectVehicle.isReverseSegment()) {
            double lower = lowerLimit1;
            double upper = upperLimit1;
            lowerLimit1 = subjectVehicle.getLink().getSegment(subjectVehicle.getSegmentIndex()).getLength() - upper;
            upperLimit1 = subjectVehicle.getLink().getSegment(subjectVehicle.getSegmentIndex()).getLength() - lower;
        }
        //____________________
//        boolean accident = false;
//        if (rand.nextInt() % (int) Parameters.encounterPerAccident == 0) {
//            accident = true;
//        }
//        ArrayList<Pedestrian> pedestriansToRemove = new ArrayList<>();
        for (Pedestrian pedestrian : pedestrianList) {
            double objpos = pedestrian.getInitPos();
            if (lowerLimit1 < objpos && objpos < upperLimit1) {
//                if (!accident) {
//                    return false;
//                }
                return false;

//                pedestrian.getSegment().setAccidentCount(pedestrian.getSegment().getAccidentCount() + 1);
//                pedestrian.inAccident = true;
//                pedestriansToRemove.add(pedestrian);
            }
        }
//        pedestrianList.removeAll(pedestriansToRemove);

        if (subjectVehicle.getDlcModel() == Parameters.DLC_MODEL.NAIVE_MODEL) {
            return true;
        }
        if (subjectVehicle.getDlcModel() == Parameters.DLC_MODEL.GIPPS_MODEL || subjectVehicle.getDlcModel() == Parameters.DLC_MODEL.GHR_MODEL) {
            /*
             * checking for feasibility of changing lane
             */
            Vehicle targetLeader = subjectVehicle.getProbableLeader();
            Vehicle targetFollower = subjectVehicle.getProbableFollower();
        /*
         lane changing feasibility variable
         true means not feasible; false means feasible
         Gipp's method is used for computing feasibility using velocity
         */
            boolean isNotFeasible;
        /*
         gap acceptance probability calculation variable
         https://www.civil.iitb.ac.in/tvm/nptel/534_LaneChange/web/web.html#x1-50002.2
         this model is used for computing probability
         */
            double probabilty;
            if (targetLeader != null) {
                // lane changing feasibility calculation
                if (subjectVehicle.getDlcModel() == Parameters.DLC_MODEL.GIPPS_MODEL) {
                    double speedWRTLeader = Vehicle.getSpeedForBraking(targetLeader, subjectVehicle);
                    double decelerationWRTLeader = (speedWRTLeader - subjectVehicle.getSpeed()) / Vehicle.TIME_STEP;
                    isNotFeasible = decelerationWRTLeader < subjectVehicle.getMaxBraking();
                } else if (subjectVehicle.getDlcModel() == Parameters.DLC_MODEL.GHR_MODEL) {
                    isNotFeasible = Vehicle.getAccelerationGHRModel(targetLeader, subjectVehicle) < subjectVehicle.getMaxBraking();
                } else {
                    // dummy
                    assert false;
                    System.out.println("should not come here");
                    isNotFeasible = false;
                }

                // gap acceptance probability calculation
                double leadTimeGap = Vehicle.getGap(targetLeader, subjectVehicle) / subjectVehicle.getSpeed();
                if (leadTimeGap > Vehicle.SAFE_TIME_GAP) {
                    probabilty = 1 - Math.exp(-Vehicle.LAMBDA * (leadTimeGap - Vehicle.SAFE_TIME_GAP));
                } else {
                    probabilty = 0; // acceleration model is also incorporated
                }
            } else {
                isNotFeasible = false;
                probabilty = 1;
            }

            if (targetFollower != null) {
                // lane changing feasibility calculation

                double lagGap = Vehicle.getGap(subjectVehicle, targetFollower);
                double modifiedLagGap = lagGap + subjectVehicle.getSpeed() - targetFollower.getSpeed();
                double speedOfFollower = Vehicle.getSpeedForBraking(subjectVehicle, targetFollower, modifiedLagGap);

                if (subjectVehicle.getDlcModel() == Parameters.DLC_MODEL.GIPPS_MODEL) {
                    double decelerationOfFollower = (speedOfFollower - targetFollower.getSpeed()) / Vehicle.TIME_STEP;
                    isNotFeasible = isNotFeasible || (decelerationOfFollower < targetFollower.getMaxBraking());
                } else if (subjectVehicle.getDlcModel() == Parameters.DLC_MODEL.GHR_MODEL) {
                    isNotFeasible = isNotFeasible || (Vehicle.getAccelerationGHRModel(subjectVehicle, targetFollower) < targetFollower.getMaxBraking());
                }

                // gap acceptance probability calculation
                double lagTimeGap = modifiedLagGap / targetFollower.getSpeed();
                if (lagTimeGap > Vehicle.SAFE_TIME_GAP) {
                    probabilty *= 1 - Math.exp(-Vehicle.LAMBDA * (lagTimeGap - Vehicle.SAFE_TIME_GAP));
                } else {
                    probabilty = 0; // acceleration model is also incorporated
                }
            }

            if (isNotFeasible) {
                // lane changing not feasible
                return false;
            }
            /*
             * feasibility checking ended. after this lane changing is feasible
             */

            /*
             * check for gap acceptance (probabilistic method)
             */
            double r = rand.nextDouble();
            return r < probabilty;
        }

        if (subjectVehicle.getDlcModel() == Parameters.DLC_MODEL.MOBIL_MODEL) {
            Vehicle target_follower = subjectVehicle.getProbableFollower();
            Vehicle target_leader = subjectVehicle.getProbableLeader(); // this was previously the leader of vehicle_n

            double b_safe = -4; // m/s^2
            double p = 1;
            double a_th = 0.1; // m/s^2

            double a_s = Vehicle.getIDMAcceleration(leader, subjectVehicle);
            double a_s_1 = Vehicle.getIDMAcceleration(subjectVehicle, follower);
            double a_n = Vehicle.getIDMAcceleration(target_leader, target_follower);

            double a_s_prime = Vehicle.getIDMAcceleration(target_leader, subjectVehicle);
            double a_s_1_prime = Vehicle.getIDMAcceleration(leader, follower);
            double a_n_prime = Vehicle.getIDMAcceleration(subjectVehicle, target_follower);

            boolean condition1 = a_n_prime >= b_safe;
            boolean condition2 = ((a_s_prime - a_s) + p * (a_n_prime - a_n + a_s_1_prime - a_s_1)) > a_th;

            return condition1 && condition2;
        }

        return true;
    }

}
