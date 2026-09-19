package thesisfinal;

import javax.swing.*;
import java.awt.*;
import java.io.*;
import java.util.*;
import java.util.logging.Level;
import java.util.logging.Logger;
import static thesisfinal.Constants.*;
import static thesisfinal.Parameters.*;
import static thesisfinal.Utilities.*;

import org.apache.commons.math3.distribution.PoissonDistribution;

import static java.lang.Math.abs;
import static thesisfinal.LinkSegmentOrientation.getLinkAndSegmentOrientation;


public class Processor {

    private final ArrayList<Node> nodeList = new ArrayList<>();
    private final ArrayList<Link> linkList = new ArrayList<>();
    private final LinkedList<Pedestrian> pedestrians = new LinkedList<>();

    private final LinkedList<Vehicle> vehicleList = new LinkedList<>();
    private final LinkedList<Object> objectList = new LinkedList<>();

    private final ArrayList<Demand> demandList = new ArrayList<>();
    private final ArrayList<Path> pathList = new ArrayList<>();
    private final ArrayList<Integer> nextGenerationTime = new ArrayList<>();
    private final ArrayList<Integer> numberOfVehiclesToGenerate = new ArrayList<>();
    private final ArrayList<Node> intersectionList = new ArrayList<>();
    private final Random random = Parameters.random;
    private int vehicleId = 0;
    private int objectId = 0; 
    private int pedestrianId = 0;
    private boolean startAlongPed = Parameters.alongPedestrianMode;
    private Point2D midPoint;

    // variables for roadside objects
    private static int numberOfVehicles = 0;
	private static int numberOfObjects = 0;	
	private static int numberOfStandingPedestrians = 0;
	private static int numberOfParkedCars = 0;
	private static int numberOfParkedRickshaws = 0;
	private static int numberOfParkedCNGS = 0;

    public Processor() {
        readNetwork();
        readPath();
        readDemand();
        addPathToDemand();

        Statistics s = new Statistics(demandList.size());
        File stats_folder = new File("statistics");

        if (!stats_folder.mkdir()) {
            // System.out.println("Stats dir not created");
        }

        for (Demand demand1 : demandList) {
            nextGenerationTime.add(1);

            double demand = demand1.getDemand();          //returns number of vehicle
            double demandRatio = 3600 / demand;

            if (demandRatio > 1) {
                numberOfVehiclesToGenerate.add(1);
            } else {
                numberOfVehiclesToGenerate.add((int) Math.round(1 / demandRatio));
            }
        }

    }

    public Point2D getMidPoint() {
        return midPoint;
    }

    public LinkedList<Pedestrian> getPedestrians() {
        return pedestrians;
    }

    public LinkedList<Vehicle> getVehicleList() {
        return vehicleList;
    }

    public LinkedList<Object> getObjectList(){
        return objectList;
    }

    public ArrayList<Node> getNodeList() {
        return nodeList;
    }

    public ArrayList<Link> getLinkList() {
        return linkList;
    }

    public void autoProcess() {
        while (true) {
            if (Parameters.simulationStep < Parameters.simulationEndTime) {
                try {
                    runAtEachTimeStep();
                } catch (IOException ex) {
                    ex.printStackTrace();
                }

            } else if (Parameters.simulationStep == Parameters.simulationEndTime) {
                if (Constants.PRINT_RESULT) {
                    generateStatistics();
                }
                break;
            }
            Parameters.simulationStep++;
        }
    }

    public void manualProcess(JFrame jFrame) {
        if (Parameters.simulationStep < Parameters.simulationEndTime) {
            if (!Parameters.TRACE_MODE) {
                try {
                    runAtEachTimeStep();
                } catch (IOException ex) {
                    ex.printStackTrace();
                }
            }
            jFrame.repaint();
        } else if (Parameters.simulationStep == Parameters.simulationEndTime) {
            if (Constants.PRINT_RESULT) {
                generateStatistics();
            }
            JSlider jSlider = new JSlider(JSlider.VERTICAL, 0, 0, 0);
            jFrame.getContentPane().add(jSlider, BorderLayout.WEST);
            jFrame.repaint();
            jFrame.revalidate();
        }
        Parameters.simulationStep++;
    }

    private void runAtEachTimeStep() throws IOException {
        if (Parameters.simulationStep % Parameters.SIGNAL_CHANGE_DURATION == 0) {
            controlSignal();
        }

        if (startAlongPed) {
            if (Parameters.simulationStep % 20 == 0) {
                if (Parameters.simulationStep != 0)
                    Parameters.alongPedestrianMode = !Parameters.alongPedestrianMode;
            }
        }
        

        if (Parameters.acrossPedestrianMode) {
            removeOldPedestrians();

            generateNewPedestrians();

            movePedestrians();
        }
        // System.out.println(vehicleList.size());
        removeOldVehicles();

        generateNewVehicles();

        if(Parameters.alongPedestrianMode)
        {
            generateNewAlongPedestrians();
        }

        

//        generateSpecificVehicles();

        moveVehicles();

        if(Parameters.OBJECT_MODE)
        {
            generateNewObjects();
            removeOldObjects();
            // if(simulationStep%20 == 0)
            // {
            //     System.out.println(numberOfStandingPedestrians+" "+numberOfParkedCars+" "+numberOfParkedRickshaws+" "+numberOfParkedCNGS);
            // }
        }

    }

    private void generateStatistics() {

        double[] sensorVehicleCount = new double[linkList.size()];
        double[] sensorVehicleAvgSpeed = new double[linkList.size()];
        double[] accidentCount = new double[linkList.size()];
        double[] nearCrashCount = new double[linkList.size()];
        double[] avgSpeedInLink = new double[linkList.size()];
        double[] avgWaitingTimeInLink = new double[linkList.size()]; // in percentage

        int index, index2;

        for (index = 0; index < linkList.size(); index++) {
            int waitingInSegment = 0, leaving = 0;
            for (index2 = 0; index2 < linkList.get(index).getNumberOfSegments(); index2++) {
                sensorVehicleCount[index] += linkList.get(index).getSegment(index2).getSensorVehicleCount();
                sensorVehicleAvgSpeed[index] += linkList.get(index).getSegment(index2).getSensorVehicleAvgSpeed() * 3600.0 / 1000;
                accidentCount[index] += linkList.get(index).getSegment(index2).getAccidentCount();
                nearCrashCount[index] += linkList.get(index).getSegment(index2).getNearCrashCount();
                avgSpeedInLink[index] += linkList.get(index).getSegment(index2).getAvgSpeedInSegment();
                waitingInSegment += linkList.get(index).getSegment(index2).getTotalWaitingTime();
                leaving += linkList.get(index).getSegment(index2).getLeavingVehicleCount();
            }
            // this means the flow rate on a link (vehicle/hour)
            sensorVehicleCount[index] = (int) Math.round(sensorVehicleCount[index] / index2 * 3600 / Parameters.simulationEndTime);

            sensorVehicleAvgSpeed[index] /= index2;
            avgSpeedInLink[index] /= index2;
            avgWaitingTimeInLink[index] = leaving > 0 ? 1.0 * waitingInSegment / leaving : 0;
        }

        /*//plots graph
        SensorVehicleCountPlot p1 = new SensorVehicleCountPlot(sensorVehicleCount);
        SensorVehicleAvgSpeedPlot p2 = new SensorVehicleAvgSpeedPlot(sensorVehicleAvgSpeed);
        AccidentCountPlot p3 = new AccidentCountPlot(accidentCount);

        printData(sensorVehicleCount, "statistics/flow_rate.csv");
        printData(sensorVehicleAvgSpeed, "statistics/sensor_speed.csv");
        printData(accidentCount, "statistics/accident.csv");
        printData(nearCrashCount, "statistics/near_crash.csv");
        printData(avgSpeedInLink, "statistics/avg_speed_link.csv");
        printData(avgWaitingTimeInLink, "statistics/avg_waiting_link.csv");*/

        

        double[] percentageOfWaiting = new double[Constants.TYPES_OF_CARS];
        for (Vehicle vehicle : vehicleList) {
            if (vehicle.isInIntersection()) {
                vehicle.increaseTraveledDistance(vehicle.getDistanceInIntersection());
            } else {
                vehicle.increaseTraveledDistance(vehicle.getDistanceInSegment());
            }

            Statistics.totalTravelTime[vehicle.getType()] += vehicle.getTravelTime();
            Statistics.waitingTime[vehicle.getType()] += vehicle.getWaitingTime();
            vehicle.calculateStatisticsAtEnd();            

        }

        
        // overall statistics
        double overall_avg_speed = 0;
		double overall_waiting_time = 0;
		int total_vehicle_count=0;

        // motorized statistics
        double motorized_avg_speed = 0;
        double motorized_waiting_time = 0;
        int motorized_vehicle_count = 0;

        // non-motorized statistics
        double non_motorized_avg_speed = 0;
        double non_motorized_waiting_time = 0;
        int non_motorized_vehicle_count = 0;

        for (int i = 0; i < Constants.TYPES_OF_CARS; i++) {
            if (Statistics.noOfVehicles[i] != 0) {
                if(i!=12)
                {
                    overall_avg_speed += Statistics.avgSpeedOfVehicle[i];
                    overall_waiting_time += Statistics.waitingTime[i];
                    total_vehicle_count += Statistics.noOfVehicles[i];

                    if(i<3)
                    {
                        non_motorized_avg_speed += Statistics.avgSpeedOfVehicle[i];
                        non_motorized_waiting_time += Statistics.waitingTime[i];
                        non_motorized_vehicle_count += Statistics.noOfVehicles[i];
                    }
                    else
                    {
                        motorized_avg_speed += Statistics.avgSpeedOfVehicle[i];
                        motorized_waiting_time += Statistics.waitingTime[i];
                        motorized_vehicle_count += Statistics.noOfVehicles[i];
                    }
                }
                
                Statistics.avgSpeedOfVehicle[i] /= Statistics.noOfVehicles[i];
                Statistics.avgSpeedOfVehicle[i] *= 3.6;
                percentageOfWaiting[i] = 100.0 * Statistics.waitingTime[i] / Statistics.totalTravelTime[i];
            }
        }

        overall_avg_speed /= total_vehicle_count;
        overall_waiting_time /= total_vehicle_count;

        motorized_avg_speed /= motorized_vehicle_count;
        motorized_waiting_time /= motorized_vehicle_count;

        non_motorized_avg_speed /= non_motorized_vehicle_count;
        non_motorized_waiting_time /= non_motorized_vehicle_count;

        double shankar_palashi_car = 0;
        double shankar_palashi_bike = 0;
        double palashi_shankar_car = 0;
        double palashi_shankar_bike = 0;

        int cnt = 0;
        for(int i=4; i<=6;i++)
        {
            shankar_palashi_car += Statistics.tripTime[1][i];
            cnt += Statistics.noOfVehiclesCompletingTrip[1][i];
        }
        shankar_palashi_car /= cnt;        
        shankar_palashi_bike = Statistics.tripTime[1][3]/Statistics.noOfVehiclesCompletingTrip[1][3];

        // System.out.println(cnt+ " " +Statistics.noOfVehiclesCompletingTrip[1][3]);

        cnt = 0;
        for(int i=4; i<=6;i++)
        {
            palashi_shankar_car += Statistics.tripTime[0][i];
            cnt += Statistics.noOfVehiclesCompletingTrip[0][i];
        }
        // System.out.println(cnt);
        palashi_shankar_car /= cnt;
        palashi_shankar_bike = Statistics.tripTime[0][3]/Statistics.noOfVehiclesCompletingTrip[0][3];

        // System.out.println(cnt+ " " +Statistics.noOfVehiclesCompletingTrip[0][3]);
        


        System.out.println("speed: " + overall_avg_speed);
        System.out.println("waiting time: " + overall_waiting_time);
        System.out.println("motorized speed: " + motorized_avg_speed);
        System.out.println("motorized waiting time: " + motorized_waiting_time);
        System.out.println("non-motorized speed: " + non_motorized_avg_speed);
        System.out.println("non-motorized waiting time: " + non_motorized_waiting_time);

        if (Parameters.FT_METHOD == 4) {
            Statistics.printGapErrorStatistics();
        }

        Statistics.printSafetyStatistics();
        
        // System.out.println("sp_car: " + shankar_palashi_car);
        // System.out.println("sp_bike: " + shankar_palashi_bike);
        // System.out.println("ps_car: " + palashi_shankar_car);
        // System.out.println("ps_bike: " + palashi_shankar_bike);        

        

        printData(Statistics.avgSpeedOfVehicle, "statistics/avg_speed_vehicle.csv");
        printData(percentageOfWaiting, "statistics/waiting_percentage_vehicle.csv");
        printData(Statistics.noOfGeneratedVehicles, "statistics/generated_vehicles.csv");

        double[] aggregatedTotalTripTime = new double[Constants.TYPES_OF_CARS];
        double[] aggregatedAvgTripTime = new double[Constants.TYPES_OF_CARS];
        double[] aggregatedTotalTripComplete = new double[Constants.TYPES_OF_CARS];
        double[] aggregatedTotalFuelConsumption = new double[Constants.TYPES_OF_CARS];
        double[] aggregatedAvgFuelConsumption = new double[Constants.TYPES_OF_CARS];
        double[] aggregatedAvgCollision = new double[Constants.TYPES_OF_CARS];
        double[] aggregatedAvgAccident = new double[Constants.TYPES_OF_CARS];
        double[] totalNumberOfCollisionAndAccident = {Statistics.noOfCollisions, Statistics.noOfAccidents, vehicleId, pedestrianId};


        for (int j = 0; j < Constants.TYPES_OF_CARS; j++) {
            for (int i = 0; i < demandList.size(); i++) {
                aggregatedTotalTripComplete[j] = aggregatedTotalTripComplete[j] +  Statistics.noOfVehiclesCompletingTrip[i][j];
                aggregatedAvgCollision[j] = aggregatedAvgCollision[j] + Statistics.noCollisionsPerDemand[i][j];
                aggregatedAvgAccident[j] = aggregatedAvgAccident[j] + Statistics.noAccidentsPerDemand[i][j];
                aggregatedTotalFuelConsumption[j] = aggregatedTotalFuelConsumption[j] + Statistics.totalFuelConsumption[i][j];
                aggregatedTotalTripTime[j] = aggregatedTotalTripTime[j] + 1.0 * Statistics.tripTime[i][j];
            }
        }

        for (int i = 0; i < Constants.TYPES_OF_CARS; i++) {
            aggregatedAvgTripTime[i] = (aggregatedTotalTripTime[i] / aggregatedTotalTripComplete[i]) / (60.0 / Constants.TIME_STEP);
            aggregatedAvgFuelConsumption[i] = aggregatedTotalFuelConsumption[i] / aggregatedTotalTripComplete[i];
        }

        printData(aggregatedTotalTripComplete, "statistics/agg_total_trip_complete.csv");
        printData(aggregatedAvgTripTime, "statistics/agg_avg_tt.csv");
        printData(aggregatedAvgFuelConsumption, "statistics/agg_avg_fuel.csv");
        printData(aggregatedAvgCollision, "statistics/agg_avg_collision.csv");
        printData(aggregatedAvgAccident, "statistics/agg_avg_accident.csv");
        printData(totalNumberOfCollisionAndAccident, "statistics/agg_total_collision.csv");


        double[][] avgFuelConsumption = new double[demandList.size()][Constants.TYPES_OF_CARS];
        double[][] avgTripTime = new double[demandList.size()][Constants.TYPES_OF_CARS];

        int no_of_trip_times = Math.min(Parameters.NO_OF_ROUTES_FOR_STAT, demandList.size());
        for (int i = 0; i < no_of_trip_times; i++) {
            for (int j = 0; j < Constants.TYPES_OF_CARS; ++j) {
                avgFuelConsumption[i][j] = Statistics.totalFuelConsumption[i][j]
                        / Statistics.noOfVehiclesCompletingTrip[i][j];
                avgTripTime[i][j] = (1.0 * Statistics.tripTime[i][j] / Statistics.noOfVehiclesCompletingTrip[i][j]) / (60.0 / Constants.TIME_STEP);
            }
            printData(avgFuelConsumption[i], "statistics/fuel" + i + ".csv");
            printData(avgTripTime[i], "statistics/avg_tt" + i + ".csv");
            printData(Statistics.noOfVehiclesCompletingTrip[i], "statistics/trip_complete" + i + ".csv");
            printData(Statistics.noCollisionsPerDemand[i], "statistics/collisions" + i + ".csv");
        }

        if (Parameters.acrossPedestrianMode) {
//            printData(accidentCount, "statistics/accident.csv");
            // System.out.println("Total generated pedestrians: " + pedestrianId);
            // System.out.println("Total # of accidents:  " + Statistics.noOfAccidents);
        }

        // flow rate statistics
        printData(Statistics.flow, "statistics/flow.csv");

        // System.out.println("Total generated vehicles: " + vehicleId);

        // System.out.println("Total # of collisions: " + Statistics.noOfCollisions);
        // System.out.println("Ended at " + new Date());
    }

    private void printData(double[] data, String filename) {
        try (PrintWriter writer = new PrintWriter(new FileOutputStream(filename, true))) {
            for (double d : data) {
                writer.printf("%.3f,", d);
            }
            writer.println();
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
    }

    private void removeOldPedestrians() {
        ArrayList<Pedestrian> objectsToRemove = new ArrayList<>();
        for (Pedestrian pedestrian : pedestrians) {
            System.out.printf("");
            if (pedestrian.isToRemove()) {
                objectsToRemove.add(pedestrian);
            }
        }
        pedestrians.removeAll(objectsToRemove);
    }

    private void generateNewPedestrians() {
        for (Link link : linkList) {
            // across_ped_percentage == number of ped per one hour
            // double pedestrian_per_hour =  Parameters.ACROSS_PEDESTRIAN_PER_HOUR * Constants.TIME_STEP;
            // System.out.println(pedestrian_per_hour);
            
            int pedestrian_count = road_crossing_ped_poisson.sample();
            // System.out.println(pedestrian_count);
            for (int x = 0; x < pedestrian_count; x++) {                
                    int randomSegmentId = Math.abs(random.nextInt()) % link.getNumberOfSegments();
                    Segment randomSegment = link.getSegment(randomSegmentId);

                    double min = 9;
                    double max = (int) (randomSegment.getLength() - 9);
                    double distance1 = (max-min)/5 + min;
                    double distance2 = max - (max-min)/5;                    
                    if ((max - min) + 1 <= 0) {
                        continue;
                    }
                    

                    // pedestrian generation model. Most road-crossing pedestrians are generated on the two ends of the road.
                    Double[] probabilities = {0.4, 0.2, 0.4};                    
                    double rand = random.nextDouble(), randomPos;
                    if(rand <= probabilities[0]) randomPos = random.nextDouble(min, distance1);
                    else if(rand <= probabilities[0]+probabilities[1]) randomPos = random.nextDouble(distance1, distance2);
                    else randomPos = random.nextDouble(distance2, max); 

                    // /int randomPos = random.nextInt((max - min) + 1) + min;
                    double randomObjSpeed = (Math.abs(random.nextInt()) % 2) / 10.0 + 0.05;
                    boolean bo = random.nextBoolean();
                    int strip;
                    if (!bo) {
                        strip = randomSegment.numberOfStrips() - 1;
                    } else {
                        strip = 0;
                    }
                    Pedestrian pedestrian = new Pedestrian(pedestrianId++, randomSegment, strip, randomPos, randomObjSpeed);
                    pedestrians.add(pedestrian);
                
            }
        }
    }

    private void movePedestrians() {
        for (Pedestrian pedestrian : pedestrians) {
            if (pedestrian.hasCrossedRoad()) {
                pedestrian.setToRemove(true);
            } else {
                if (!pedestrian.moveForward()) {
                    if (!pedestrian.moveLengthWise()){
                        if (pedestrian.isStuck()) {
                            pedestrian.cleanUp();
                        }
                    }

                }
                pedestrian.printPedestrianDetails();
            }
        }
    }

    private void generateAnObject(Link link, int objectType){

        Double[] objectLengths = {STANDING_PEDESTRIAN_LENGTH, PARKED_CAR_LENGTH, PARKED_RICKSHAW_LENGTH, PARKED_CNG_LENGTH}; // lengths of the objects
        Double[] objectWidths = {STANDING_PEDESTRIAN_WIDTH, PARKED_CAR_WIDTH, PARKED_RICKSHAW_WIDTH, PARKED_CNG_WIDTH}; // widths of the objects


        int randomSegmentID = Math.abs(random.nextInt()) % link.getNumberOfSegments();
        Segment randomSegment = link.getSegment(randomSegmentID);
        int min = 9;
        int max = (int) (randomSegment.getLength() - 9);
        if ((max - min) + 1 <= 0) {
            return;
        }
        int randomInitPos = random.nextInt((max - min) + 1) + min;			
        boolean reverseDirection = random.nextBoolean();

        double distanceFromFootpath = Utilities.getRandomFromMultipleGaussianDistributionOfObjectsBlockage(objectType) - objectWidths[objectType-1]; // sampling blockage from Gaussian distribution
        double width = distanceFromFootpath + objectWidths[objectType-1];

        int numberOfOccupiedStrips;
        numberOfOccupiedStrips = (int) Math.ceil(width/stripWidth);
		// if (width%stripWidth==0) {
		// 	numberOfOccupiedStrips = (int) (width/stripWidth);
		// }
		// else {
		// 	numberOfOccupiedStrips = (int) (width/stripWidth) + 1;
		// }
		if (!reverseDirection) {
			for (int i=0; i<=numberOfOccupiedStrips; i++) { //Started from 0, as we want to occupy the footpath strip as well.
				if (!randomSegment.getStrip(i).hasGapForAddingObject(objectLengths[objectType-1], randomInitPos)) {
                    return; //There's no gap for adding object
                }
			}
		} else {
			for (int i=0; i<=numberOfOccupiedStrips; i++) { //Started from 0, as we want to occupy the footpath strip as well.
				if (!randomSegment.getStrip(randomSegment.numberOfStrips() - 1 - i).hasGapForAddingObject(objectLengths[objectType-1], randomInitPos)) {
                    return; //There's no gap for adding object
                }
			}
		}
        

        // int firstStripIndex = (int)Math.ceil((distanceFromFootpath-objectWidths[objectType-1])/stripWidth);
        // int lastStripIndex = (int)Math.ceil(distanceFromFootpath/stripWidth);

        // if(reverseDirection)
        // {
        //     firstStripIndex = randomSegment.numberOfStrips() - 1 - lastStripIndex;
        //     lastStripIndex = randomSegment.numberOfStrips() - 1 - firstStripIndex;
        // }


        // for(int i=firstStripIndex;i<=lastStripIndex;i++)
        // {
        //     if (!randomSegment.getStrip(i).hasGapForAddingObject(objectLengths[objectType-1], randomInitPos)) {
        //         return; //There's no gap for adding object
        //     }
        // }


        // if (reverseDirection) {
        //     if (!randomSegment.getStrip(randomSegment.numberOfStrips()-1).hasGapForAddingObject(objectLengths[objectType-1], randomInitPos)) {
        //         return; //There's no gap for adding object
        //     }
        // } else {
        //     if (!randomSegment.getStrip(0).hasGapForAddingObject(objectLengths[objectType-1], randomInitPos)) {
        //         return; //There's no gap for adding object
        //     }
        // }			

        Object object = new Object( objectId++, objectType, simulationStep, link, randomSegment, randomSegmentID, randomInitPos, reverseDirection, distanceFromFootpath);
        objectList.add(object);
        numberOfObjects++;
        if(objectType == 1)numberOfStandingPedestrians++;
        else if(objectType == 2)numberOfParkedCars++;
        else if(objectType == 3)numberOfParkedRickshaws++;
        else numberOfParkedCNGS++;
    }

    private void generateNewObjects () {
		for (Link link : linkList) {			
            
            Random random = new Random();
            double stdev = 4;
            double factor=0.1;
            double randomGaussian = 0;

            // We generate vehicle using cumulative Gaussian dsitribution. As number of vehicles increase, the chance of generating new vehicles decerase

            // generating standing pedestrians
            randomGaussian = 1 - cumulativeDistributionFunction(numberOfStandingPedestrians, AVG_NUMBER_OF_STANDING_PEDESTRIANS, 10);
            if(random.nextDouble() < factor*3*randomGaussian)
            {
                generateAnObject(link, 1);
            }

            // generating parked cars
            randomGaussian = 1 - cumulativeDistributionFunction(numberOfParkedCars, AVG_NUMBER_OF_PARKED_CARS, 10);
            if(random.nextDouble() < factor*randomGaussian)
            {
                generateAnObject(link, 2);
            }

            // generating parked rickshaws
            randomGaussian = 1 - cumulativeDistributionFunction(numberOfParkedRickshaws, AVG_NUMBER_OF_PARKED_RICKSHAWS, 10);
            if(random.nextDouble() < factor*4*randomGaussian)
            {
                generateAnObject(link, 3);
            }

            // generating parked CNGs
            randomGaussian = 1 - cumulativeDistributionFunction(numberOfParkedCNGS, AVG_NUMBER_OF_PARKED_CNGS, 10);
            if(random.nextDouble() < factor*7*randomGaussian)
            {
                generateAnObject(link, 4);
            }

		}
	}

    private void removeOldObjects () {

		ArrayList<Object> objectsToRemove = new ArrayList<>();
		for (Object object : objectList) {	
            
            if(simulationStep - object.getParkingStartTime() < object.getParkingTime())  // parking time is not over yet
            {
                continue;
            }
			
           // parking time over, the object will move on

			//for standing pedestrians
			if (object.objectType==1) {

                numberOfStandingPedestrians--;
                double width = object.getDistanceFromFootpath() + STANDING_PEDESTRIAN_WIDTH;
                removeRectangularObject(object, width);
                objectsToRemove.add(object);
			}

			//for parked cars
			if (object.objectType==2) {

                numberOfParkedCars--;
                double width = object.getDistanceFromFootpath() + PARKED_CAR_WIDTH;
                removeRectangularObject(object, width);
                objectsToRemove.add(object);
			}

			//for parked rickshaws
			if (object.objectType==3) {

                numberOfParkedRickshaws--;
                double width = object.getDistanceFromFootpath() + PARKED_RICKSHAW_WIDTH;
                removeRectangularObject(object, width);
                objectsToRemove.add(object);
			}

			//for parked CNGs
			if (object.objectType==4) {

                numberOfParkedCNGS--;
                double width = object.getDistanceFromFootpath() + PARKED_CNG_WIDTH;
                removeRectangularObject(object, width);
                objectsToRemove.add(object);
			}
		}

		numberOfObjects = numberOfObjects - objectsToRemove.size();
		objectList.removeAll(objectsToRemove);
	}

    private static void removeRectangularObject(Object object, double width) {
		int numberOfOccupiedStrips;
		if (width%stripWidth==0) {
			numberOfOccupiedStrips = (int) (width/stripWidth);
		}
		else {
			numberOfOccupiedStrips = (int) (width/stripWidth) + 1;
		}
		if (!object.isReverseSegment()) {
			for (int i=0; i<=numberOfOccupiedStrips; i++) { //Started from 0, as footpath strip should be freed as well.
				object.getSegment().getStrip(i).delObject(object);
			}
		} else {
			for (int i=0; i<=numberOfOccupiedStrips; i++) { //Started from 0, as footpath strip should be freed as well.
				object.getSegment().getStrip(object.getSegment().numberOfStrips()-1 - i).delObject(object);
			}
		}
	}

    private int randomVehiclePath(int numberOfPaths) {
        return abs(random.nextInt()) % numberOfPaths;
    }

    private double randomVehicleSpeed() {
        return (random.nextInt(10) + 1); // speed in 1 to 10
        //return random.nextInt((int) maximumSpeed) + 1;
    }

    private double constantVehicleSpeed() {
        return 0;
    }

    private int constantVehicleType() {
        return 4;
    }

    private int randomVehicleType() {
        // first 3 are human powered
        int ratio = random.nextInt(101);
        int type;
        if (ratio < Parameters.slowVehiclePercentage) {
            type = random.nextInt(3); // first 3 are slow human powered vehicle
        } else if (ratio < Parameters.slowVehiclePercentage + Parameters.mediumVehiclePercentage) {
            type = random.nextInt(5); // types of medium speed vehicle = 5
            type += 7; // last
        } else {
            type = random.nextInt(4); // types of high speed vehicle = 4
            type += 3; // offset
        }

        return type;
    }

    private int distributedVehicleType() {
        int ratio = random.nextInt(101);
        // System.out.println(Parameters.slowVehiclePercentage + " " + Parameters.mediumVehiclePercentage+ " " + Parameters.fastVehiclePercentage);
        if (ratio < Parameters.slowVehiclePercentage) {
            int ratioN = random.nextInt(101);
            if (ratioN < 9) {
                return 0; // bicycle
            } else if (ratioN < 98) {
                return 1; // rickshaw
            } else {
                return 2; // van/cart
            }
        } else if (ratio < Parameters.slowVehiclePercentage + Parameters.mediumVehiclePercentage) {
            int ratioN = random.nextInt(101);
            if (ratioN < 83) {
                return 7; // cng
            } else if (ratioN < 98) {
                return 8 + random.nextInt(2); // bus
            } else {
                return 10 + random.nextInt(2); // truck
            }
        } else {
            int ratioN = random.nextInt(101);
            if (ratioN < 88) {
                return 3; // bike
            } else {
                return 4 + random.nextInt(3); // car
            }
        }
    }

    private int distributedVehicleTypeMiami() {
        int ratio = random.nextInt(101);
        if (ratio < Parameters.slowVehiclePercentage) {
            return 0; // always bicycle
        } else if (ratio < Parameters.slowVehiclePercentage + Parameters.mediumVehiclePercentage) {
            return 8 + random.nextInt(2); // always bus
        } else {
            int ratioN = random.nextInt(101);
            if (ratioN < 12) {
                return 3; // bike
            } else {
                return 4 + random.nextInt(3); // car
            }
        }
    }

    private int distributedVehicleTypeBDNew() {
        int ratio = random.nextInt(101);
        if (ratio < 26) {
            return 1; // rickshaw
        } else if (ratio < 26 + 23) {
            return 7; // CNG
        } else if (ratio < 26 + 23 + 30) {
            return 3;
        } else if (ratio < 26 + 23 + 30 + 18) {
            return 4 + random.nextInt(3);
        } else {
            return 8 + random.nextInt(2); // bus
        }
    }

    private int pedestrian_vehicle_distributionType() {
        // if (Parameters.alongPedestrianMode) {
        //     int ratio = random.nextInt(101);
        //     if (ratio < Parameters.ALONG_PEDESTRIAN_PERCENTAGE) {
        //         return 12;
        //     }
        //     else {
        //         // return constantVehicleType();
        //         return distributedVehicleType();
        //     }
        // }
        // else {
        //     // return constantVehicleType();
        //     return distributedVehicleType();
        // }
        return distributedVehicleType();
    }

    private int newPedestrianVehicleDistributionType() {
        if (Parameters.alongPedestrianMode) {
            int noOfDivision = 4;
            int durationOfDivision = 25;
            int totalDuration = noOfDivision * durationOfDivision;
            int x = Parameters.simulationStep % totalDuration;
            int y = x / durationOfDivision;
            int ratio = random.nextInt(totalDuration + 1);
            if (ratio < (y + 1) * durationOfDivision) {
                return 12;
            }
            else {
                return constantVehicleType();
            }
        } else {
            return constantVehicleType();
        }
    }

    private void createAnAlongPedestrian(int demandIndex) {
        int numberOfPaths = demandList.get(demandIndex).getNumberOfPaths();
        for (int j = 0; j < numberOfVehiclesToGenerate.get(demandIndex); j++) {
            
            int pathIndex = randomVehiclePath(numberOfPaths);
//            double speed = randomVehicleSpeed();
//            double speed = constantVehicleSpeed();
//            int type = constantVehicleType();
//            int type = distributedVehicleTypeMiami();
//            int type = distributedVehicleTypeBDNew();
//            int type = distributedVehicleType();
            int type = PEDESTRIANS_ALONG_THE_ROAD_TYPE;
//            int type = newPedestrianVehicleDistributionType();
            int numberOfStrips = Utilities.numberOfStrips(type);
            Path path = demandList.get(demandIndex).getPath(pathIndex);
            Node sourceNode = nodeList.get(path.getSource());
            Link link = linkList.get(path.getLink(0));

            LinkSegmentOrientation linkSegmentOrientation = getLinkAndSegmentOrientation(sourceNode.x, sourceNode.y, link.getFirstSegment(), link.getLastSegment());
            Segment segment = linkSegmentOrientation.reverseLink ? link.getLastSegment() : link.getFirstSegment();
            
            

            int start, end;

            // for pedestrians, we sample from the distribution for the blockage
            double distanceFromFootpath = Utilities.getRandomFromMultipleGaussianDistributionOfObjectsBlockage(5);
            int stripIndex = Utilities.pedestrianBlockageStrip(distanceFromFootpath);

            if (linkSegmentOrientation.reverseSegment) {
                stripIndex = segment.lastVehicleStripIndex - stripIndex + 1;
            }  
            
            if(hasGap(type, stripIndex, segment))
            {
                createVehicle(type, linkSegmentOrientation, link, segment, demandIndex, pathIndex, stripIndex);
            }
            return;

           

//            vehicleGenNodes.get(sourceNode.getId()).add(new Vehicle())
        }
    }

    private void createAVehicle(int demandIndex) {
        int numberOfPaths = demandList.get(demandIndex).getNumberOfPaths();
        for (int j = 0; j < numberOfVehiclesToGenerate.get(demandIndex); j++) {
            
            int pathIndex = randomVehiclePath(numberOfPaths);
//            double speed = randomVehicleSpeed();
//            double speed = constantVehicleSpeed();
//            int type = constantVehicleType();
//            int type = distributedVehicleTypeMiami();
//            int type = distributedVehicleTypeBDNew();
//            int type = distributedVehicleType();
            int type = pedestrian_vehicle_distributionType();
            // System.out.println(type);
            // if(type < 3) return;
//            int type = newPedestrianVehicleDistributionType();
            int numberOfStrips = Utilities.numberOfStrips(type);
            Path path = demandList.get(demandIndex).getPath(pathIndex);
            Node sourceNode = nodeList.get(path.getSource());
            Link link = linkList.get(path.getLink(0));

            LinkSegmentOrientation linkSegmentOrientation = getLinkAndSegmentOrientation(sourceNode.x, sourceNode.y, link.getFirstSegment(), link.getLastSegment());
            Segment segment = linkSegmentOrientation.reverseLink ? link.getLastSegment() : link.getFirstSegment();
            
            

            int start, end;

            // for pedestrians, we sample from the distribution for the blockage
            if(type == PEDESTRIANS_ALONG_THE_ROAD_TYPE)
            {
                double distanceFromFootpath = Utilities.getRandomFromMultipleGaussianDistributionOfObjectsBlockage(5);
                int stripIndex = Utilities.pedestrianBlockageStrip(distanceFromFootpath);

                if (linkSegmentOrientation.reverseSegment) {
                    stripIndex = segment.lastVehicleStripIndex - stripIndex + 1;
                }  
                
                if(hasGap(type, stripIndex, segment))
                {
                    createVehicle(type, linkSegmentOrientation, link, segment, demandIndex, pathIndex, stripIndex);
                }
                return;
            }

            // other vehicles are created normally

            if (linkSegmentOrientation.reverseSegment) {
                start = segment.middleHighStripIndex;
                end = segment.lastVehicleStripIndex;
            } else {
                start = 1;
                end = segment.middleLowStripIndex;
            }         

            

            // System.out.println(start+" "+end);
            int randomStart = Utilities.randInt(start, end);

            for (int k = randomStart; k + numberOfStrips - 1 <= end; k++) {
                boolean hasGap = hasGap(type, k, segment);

                if (hasGap) {
                    createVehicle(type, linkSegmentOrientation, link, segment, demandIndex, pathIndex, k);
                    return;
                }
            }

            for (int k = end - numberOfStrips + 1; k >= randomStart; --k) {
                boolean hasGap = hasGap(type, k, segment);

                if (hasGap) {
                    createVehicle(type, linkSegmentOrientation, link, segment, demandIndex, pathIndex, k);
                    return;
                }
            }

//            vehicleGenNodes.get(sourceNode.getId()).add(new Vehicle())
        }
    }

    private boolean hasGap(int type, int startIndex, Segment segment) {
        int numberOfStrips = Utilities.numberOfStrips(type);
        double length = Utilities.getCarLength(type);
        boolean hasGap = true;
        for (int l = startIndex; l < startIndex + numberOfStrips; l++) {
            if(l<0 || l>=segment.numberOfStrips())
            {
                hasGap = false;
                continue;
            }
            if (!segment.getStrip(l).hasGapForAddingVehicle(length)) {
                hasGap = false;
            }
        }        
        return hasGap;
    }

    private void createVehicle(int type, LinkSegmentOrientation linkSegmentOrientation, Link link, Segment segment,
                               int demandIndex, int pathIndex, int stripIndex) {

        Color color = new Color(random.nextFloat(), random.nextFloat(), random.nextFloat());
        if (type == PEDESTRIANS_ALONG_THE_ROAD_TYPE) {
            color = Color.BLACK;
        }
        double startX, startY, endX, endY;
        if (linkSegmentOrientation.reverseSegment) {
            startX = segment.getEndX();
            startY = segment.getEndY();
            endX = segment.getStartX();
            endY = segment.getStartY();
        } else {
            startX = segment.getStartX();
            startY = segment.getStartY();
            endX = segment.getEndX();
            endY = segment.getEndY();
        }
        

        vehicleList.add(new Vehicle(vehicleId++, Parameters.simulationStep, type, color, demandIndex, pathIndex, 0,
                startX, startY, endX, endY, linkSegmentOrientation.reverseLink, linkSegmentOrientation.reverseSegment, link,
                segment.getIndex(), stripIndex));
        Statistics.noOfGeneratedVehicles[type] += 1;
    }

    
    private void createASpecificVehicle(int demandIndex, int type, int stripIndex, Color color) {
        int numberOfPaths = demandList.get(demandIndex).getNumberOfPaths();
        for (int j = 0; j < numberOfVehiclesToGenerate.get(demandIndex); j++) {
            int pathIndex = randomVehiclePath(numberOfPaths);
            double length = Utilities.getCarLength(type);
            int numberOfStrips = Utilities.numberOfStrips(type);
            Path path = demandList.get(demandIndex).getPath(pathIndex);
            Node sourceNode = nodeList.get(path.getSource());
            Link link = linkList.get(path.getLink(0));

            LinkSegmentOrientation linkSegmentOrientation = getLinkAndSegmentOrientation(sourceNode.x, sourceNode.y, link.getFirstSegment(), link.getLastSegment());
            Segment segment = linkSegmentOrientation.reverseLink ? link.getLastSegment() : link.getFirstSegment();

            boolean hasGap = true;
                for (int l = stripIndex; l < stripIndex + numberOfStrips; l++) {
                    if (!segment.getStrip(l).hasGapForAddingVehicle(length)) {
                        hasGap = false;
                    }
                }
                if (hasGap) {
//                    Color color = new Color(random.nextFloat(), random.nextFloat(), random.nextFloat());
//                    if (type == 12) {
//                        color = Color.BLACK;
//                    }
                    double startX, startY, endX, endY;
                    if (linkSegmentOrientation.reverseSegment) {
                        startX = segment.getEndX();
                        startY = segment.getEndY();
                        endX = segment.getStartX();
                        endY = segment.getStartY();
                    } else {
                        startX = segment.getStartX();
                        startY = segment.getStartY();
                        endX = segment.getEndX();
                        endY = segment.getEndY();
                    }

                    vehicleList.add(new Vehicle(vehicleId++, Parameters.simulationStep, type, color, demandIndex, pathIndex, 0,
                            startX, startY, endX, endY, linkSegmentOrientation.reverseLink, linkSegmentOrientation.reverseSegment,
                            link, segment.getIndex(), stripIndex));
                    return;
                }
        }
    }
    /**
     * @param meanHeadway the constant time gap between two consecutive vehicle
     * @return the time gap to be used based on various distributions
     */
    private double getNextInterArrivalGap(double meanHeadway) {
        double X;
        if (Parameters.vehicle_generation_rate == Parameters.VEHICLE_GENERATION_RATE.CONSTANT) {
            X = meanHeadway;
        } else if (Parameters.vehicle_generation_rate == Parameters.VEHICLE_GENERATION_RATE.POISSON) {
            X = meanHeadway * -Math.log(random.nextDouble());
        } else {
            X = 0;
            assert false;
        }
        return X;
    }

    private void removeOldVehicles() {
        ArrayList<Vehicle> vehiclesToRemove = new ArrayList<>();
        for (Vehicle vehicle : vehicleList) {
            if (vehicle.isToRemove()) {
                vehicle.freeStrips();
                vehiclesToRemove.add(vehicle);
                vehicle.calculateStatisticsAtEnd();
                if (writeSpeed(vehicle)) {
                    printData(vehicle.getVehicleStats().getSpeeds(), "statistics/speeds_" + vehicle.getVehicleId() + ".csv");
                    printData(vehicle.getVehicleStats().getXTrajectory(), "statistics/x_coords_" + vehicle.getVehicleId() + ".csv");
                    printData(vehicle.getVehicleStats().getYTrajectory(), "statistics/y_coords_" + vehicle.getVehicleId() + ".csv");
                }
                Statistics.saveVehicleStat(vehicle.getVehicleStats());
                vehicle.updateTripTimeStatistics();
            }
        }
        vehicleList.removeAll(vehiclesToRemove);
    }

    private void generateNewAlongPedestrians() {
        // System.out.println(ALONG_PEDESTRIAN_PER_HOUR);
        for(int i=0;i<demandList.size();i++)
        {
        
        int pedestrianCount = road_along_ped_poisson.sample();
        // int rand = random.nextInt(demandList.size());
        // System.out.println("Pedestrian count: " + pedestrianCount+ " Demand index: "+rand);
        for (int j = 0; j < pedestrianCount; j++) {
            createAnAlongPedestrian(i);
        }

    }

    }

    private void generateNewVehicles() {

        // if(simulationStep  == 5)
        // {            
        //     createAVehicle(5);
        // }
        for (int i = 0; i < demandList.size(); i++) {
            if ((int) (nextGenerationTime.get(i) / Constants.TIME_STEP) == Parameters.simulationStep) {
                int demand = demandList.get(i).getDemand();
                double meanHeadway = 3600.0 / demand;
                int nextTime;
                do {
                    createAVehicle(i);

                    double X = getNextInterArrivalGap(meanHeadway);

                    nextTime = (int) Math.round(nextGenerationTime.get(i) + X);
                    nextGenerationTime.set(i, nextTime);
                } while ((int)(nextTime / Constants.TIME_STEP) == Parameters.simulationStep);
            }
        }
    }

    private void generateSpecificVehicles() {
        int[] timePoints = {1, 1, 1, 1, 15};
        int[] types = {0, 0, 0, 0, 5};
//        int[] stripIndices = {6, 1, 2, 12, 3};
        int[] stripIndices = {2, 4, 6, 10, 3};
        Color[] colors = {Color.GREEN, Color.RED, Color.CYAN, Color.MAGENTA, Color.BLUE};

        for (int i = 0; i < timePoints.length; i++) {
            if (Parameters.simulationStep == timePoints[i]) {
                createASpecificVehicle(0, types[i], stripIndices[i], colors[i]);

            }
        }
    }

    boolean writeSpeed(Vehicle vehicle) {
        int[] ids = {1, 51, 81, 101};

//        return vehicle.getVehicleId() % 40 == 0 && vehicle.getType() != 12;
        return false;
    }

    @SuppressWarnings("Duplicates")
    private void moveVehicleAtIntersectionEnd(Vehicle vehicle) {
        int demandIndex = vehicle.getDemandIndex();
        int pathIndex = vehicle.getPathIndex();
        int linkIndexOnPath = vehicle.getLinkIndexOnPath();

        int newLinkIndex = demandList.get(demandIndex).getPath(pathIndex).getLink(linkIndexOnPath + 1);
        Link link = linkList.get(newLinkIndex);

        Segment firstSegment = link.getFirstSegment();
        Segment lastSegment = link.getLastSegment();

        LinkSegmentOrientation linkSegmentOrientation = getLinkAndSegmentOrientation(vehicle.getSegEndX(), vehicle.getSegEndY(), firstSegment, lastSegment);

        Segment enteringSegment = linkSegmentOrientation.reverseLink ? lastSegment : firstSegment;

        int stripIndex = vehicle.getCurrentIntersectionStrip().endStrip;

        boolean flag = true;
        for (int j = 0; j < vehicle.getNumberOfStrips(); j++) {
            if (!enteringSegment.getStrip(stripIndex + j).hasGapForAddingVehicle(vehicle.getLength())) {
                flag = false;
                break;
            }
        }
        if (!flag) {
            stripIndex = enteringSegment.getStripIndexInEnteringSegment(vehicle, stripIndex);
            flag = stripIndex != -1;
        }

        if (flag) {
            //check this
            vehicle.getNode().removeVehicle(vehicle);
            // should not call manualSignaling() if we want to compare performance between different CF models
//            vehicle.getNode().manualSignaling();
            vehicle.setNode(null);
            vehicle.increaseTraveledDistance(vehicle.getDistanceInIntersection() + vehicle.getLength() + Vehicle.MARGIN);
            vehicle.setDistanceInIntersection(0);

            vehicle.freeStrips();

            vehicle.setInIntersection(false);
            vehicle.setReverseLink(linkSegmentOrientation.reverseLink);
            vehicle.setReverseSegment(linkSegmentOrientation.reverseSegment);
            vehicle.linkChange(linkIndexOnPath + 1, link, enteringSegment.getIndex(), stripIndex);
            vehicle.updateSegmentEnteringData();
            if (linkSegmentOrientation.reverseSegment) {
                vehicle.setSegStartX(enteringSegment.getEndX());
                vehicle.setSegStartY(enteringSegment.getEndY());
                vehicle.setSegEndX(enteringSegment.getStartX());
                vehicle.setSegEndY(enteringSegment.getStartY());
            } else {
                vehicle.setSegStartX(enteringSegment.getStartX());
                vehicle.setSegStartY(enteringSegment.getStartY());
                vehicle.setSegEndX(enteringSegment.getEndX());
                vehicle.setSegEndY(enteringSegment.getEndY());
            }
        }
    }

    private void moveVehicleAtSegmentMiddle(Vehicle vehicle) {
        double oldDistInSegment = vehicle.getDistanceInSegment();
        boolean previous = vehicle.isPassedSensor();

        vehicle.moveVehicleInSegment();

        boolean now = vehicle.isPassedSensor();
        double newDistInSegment = vehicle.getDistanceInSegment();

        if (!previous && now) {
            vehicle.getLink().getSegment(vehicle.getSegmentIndex()).updateInformation(vehicle.getSpeed());
        }

        updateFlow(oldDistInSegment, newDistInSegment, vehicle);
    }

    void accidentLogAllVehicleAtLast(Vehicle vehicle) {
        if (vehicle.hasCollided() || vehicle.hasCausedAccident()) {
            vehicle.printAccidentLog();
        }
    }
    void accidentCheckAllVehiclesAtLast(Vehicle vehicle) {
//         if (vehicle.hasCollided()) {
//             //TODO
// //            vehicle.resetPosition();
//             vehicle.afterCollisionToDo();
//         }

        if (vehicle.hasCausedAccident()) {
            vehicle.afterAccidentToDo();
        }
    }

    @SuppressWarnings("Duplicates")
    private IntersectionStrip createIntersectionStrip(Vehicle vehicle, int oldLinkIndex, int newLinkIndex) {
        LinkSegmentOrientation linkSegmentOrientation = getLinkAndSegmentOrientation(vehicle.getSegEndX(), vehicle.getSegEndY(), linkList.get(newLinkIndex).getSegment(0), linkList.get(newLinkIndex).getSegment(linkList.get(newLinkIndex).getNumberOfSegments() - 1));

        Segment leavingSegment = vehicle.getLink().getSegment(vehicle.getSegmentIndex()); // leaving segment
        Segment enteringSegment = linkSegmentOrientation.reverseLink ? linkList.get(newLinkIndex).getLastSegment() : linkList.get(newLinkIndex).getFirstSegment(); // entering segment

        int oldStripIndex = vehicle.getStripIndex();
        int newStripIndex = vehicle.getNewStripIndex(leavingSegment, enteringSegment, linkSegmentOrientation.reverseSegment);

        double startPointX, startPointY, endPointX, endPointY;
        double w = 1 * Parameters.pixelPerFootpathStrip + (vehicle.getStripIndex() - 1) * Parameters.pixelPerStrip; //single direction so 1 footpath
        double x1 = leavingSegment.getStartX() * Parameters.pixelPerMeter;
        double y1 = leavingSegment.getStartY() * Parameters.pixelPerMeter;
        double x2 = leavingSegment.getEndX() * Parameters.pixelPerMeter;
        double y2 = leavingSegment.getEndY() * Parameters.pixelPerMeter;
        double x3 = returnX3(x1, y1, x2, y2, w);
        double y3 = returnY3(x1, y1, x2, y2, w);
        double x4 = returnX4(x1, y1, x2, y2, w);
        double y4 = returnY4(x1, y1, x2, y2, w);

        double dist1 = (x3 - vehicle.getSegEndX() * Parameters.pixelPerMeter) * (x3 - vehicle.getSegEndX() * Parameters.pixelPerMeter) + (y3 - vehicle.getSegEndY() * Parameters.pixelPerMeter) * (y3 - vehicle.getSegEndY() * Parameters.pixelPerMeter);
        double dist2 = (x4 - vehicle.getSegEndX() * Parameters.pixelPerMeter) * (x4 - vehicle.getSegEndX() * Parameters.pixelPerMeter) + (y4 - vehicle.getSegEndY() * Parameters.pixelPerMeter) * (y4 - vehicle.getSegEndY() * Parameters.pixelPerMeter);
        if (dist1 < dist2) {
            startPointX = x3;
            startPointY = y3;
        } else {
            startPointX = x4;
            startPointY = y4;
        }
        double w1;
        double x_1 = enteringSegment.getStartX() * Parameters.pixelPerMeter;
        double y_1 = enteringSegment.getStartY() * Parameters.pixelPerMeter;
        double x_2 = enteringSegment.getEndX() * Parameters.pixelPerMeter;
        double y_2 = enteringSegment.getEndY() * Parameters.pixelPerMeter;
        if (vehicle.isReverseSegment() == linkSegmentOrientation.reverseSegment) {
            w1 = 1 * Parameters.pixelPerFootpathStrip + (vehicle.getNewStripIndex(leavingSegment, enteringSegment, linkSegmentOrientation.reverseSegment) - 1) * Parameters.pixelPerStrip; //single direction so 1 footpath
        } else {
            w1 = 1 * Parameters.pixelPerFootpathStrip + (vehicle.getNewStripIndex(leavingSegment, enteringSegment, linkSegmentOrientation.reverseSegment)) * Parameters.pixelPerStrip; //single direction so 1 footpath
        }

        double x_3 = returnX3(x_1, y_1, x_2, y_2, w1);
        double y_3 = returnY3(x_1, y_1, x_2, y_2, w1);
        double x_4 = returnX4(x_1, y_1, x_2, y_2, w1);
        double y_4 = returnY4(x_1, y_1, x_2, y_2, w1);
        double dist3 = (x_3 - vehicle.getSegEndX() * Parameters.pixelPerMeter) * (x_3 - vehicle.getSegEndX() * Parameters.pixelPerMeter) + (y_3 - vehicle.getSegEndY() * Parameters.pixelPerMeter) * (y_3 - vehicle.getSegEndY() * Parameters.pixelPerMeter);
        double dist4 = (x_4 - vehicle.getSegEndX() * Parameters.pixelPerMeter) * (x_4 - vehicle.getSegEndX() * Parameters.pixelPerMeter) + (y_4 - vehicle.getSegEndY() * Parameters.pixelPerMeter) * (y_4 - vehicle.getSegEndY() * Parameters.pixelPerMeter);
        if (dist3 < dist4) {    // choosing closest segment end point from vehicle
            endPointX = x_3;
            endPointY = y_3;
        } else {
            endPointX = x_4;
            endPointY = y_4;
        }
        return new IntersectionStrip(oldLinkIndex, oldStripIndex, newLinkIndex, newStripIndex, startPointX, startPointY, endPointX, endPointY, leavingSegment, enteringSegment);
    }

    @SuppressWarnings("Duplicates")
    private void moveVehicleAtSegmentEnd(Vehicle vehicle) {
        if ((vehicle.isReverseLink() && vehicle.getLink().getSegment(vehicle.getSegmentIndex()).isFirstSegment())
                || (!vehicle.isReverseLink() && vehicle.getLink().getSegment(vehicle.getSegmentIndex()).isLastSegment())) {
            //at link end
            int demandIndex = vehicle.getDemandIndex();
            int pathIndex = vehicle.getPathIndex();
            int pathLinkIndex = vehicle.getLinkIndexOnPath();
            int lastLinkInPathIndex = demandList.get(demandIndex).getPath(pathIndex).getNumberOfLinks() - 1;
            if (pathLinkIndex == lastLinkInPathIndex) {
                //at path end
                vehicle.removeFromSimulation(true);
            } else {
                //at path middle
                int oldLinkIndex = demandList.get(demandIndex).getPath(pathIndex).getLink(pathLinkIndex);
                int newLinkIndex = demandList.get(demandIndex).getPath(pathIndex).getLink(pathLinkIndex + 1);

                Node node = vehicle.isReverseLink() ? nodeList.get(vehicle.getLink().getUpNode()) : nodeList.get(vehicle.getLink().getDownNode());

                LinkSegmentOrientation linkSegmentOrientation = getLinkAndSegmentOrientation(vehicle.getSegEndX(), vehicle.getSegEndY(), linkList.get(newLinkIndex).getSegment(0), linkList.get(newLinkIndex).getSegment(linkList.get(newLinkIndex).getNumberOfSegments() - 1));

                Segment leavingSegment = vehicle.getLink().getSegment(vehicle.getSegmentIndex()); // leaving segment
                Segment enteringSegment = linkSegmentOrientation.reverseLink ? linkList.get(newLinkIndex).getSegment(linkList.get(newLinkIndex).getNumberOfSegments() - 1) : linkList.get(newLinkIndex).getSegment(0); // entering segment

                int oldStripIndex = vehicle.getStripIndex();
                int newStripIndex = vehicle.getNewStripIndex(leavingSegment, enteringSegment, linkSegmentOrientation.reverseSegment);

                if (enteringSegment.getStripIndexInEnteringSegment(vehicle, newStripIndex) == -1) {
                    vehicle.setSpeed(0);
                    return;
                }

                if (!node.intersectionStripExists(oldLinkIndex, oldStripIndex, newLinkIndex, newStripIndex)) {

                    node.addIntersectionStrip(createIntersectionStrip(vehicle, oldLinkIndex, newLinkIndex));
                }

                if (node.isBundleActive(oldLinkIndex)) {
                    vehicle.setIntersectionStripIndex(node.getMyIntersectionStrip(oldLinkIndex, oldStripIndex, newLinkIndex, newStripIndex));
                    vehicle.setNode(node);

                    node.addVehicle(vehicle);
                    vehicle.setDistanceInIntersection(0);
                    vehicle.setInIntersection(true);
                    if (node.doOverlap(vehicle)) {
                        node.removeVehicle(vehicle);
                        vehicle.setInIntersection(false);
                        vehicle.setSpeed(0);
                    } else {
                        // in next step vehicle will start moving in intersection; so here we update statistics
                        vehicle.updateSegmentLeavingData();
                    }
                } else {
                    // here the vehicle is at segment end and the signal is red so we need to make the speed 0
                    vehicle.setSpeed(0);
                }

            }
        } else {
            //at link middle
            Link link = vehicle.getLink();
            int currentSegmentIndex = vehicle.getSegmentIndex();

            Segment enteringSegment = vehicle.isReverseLink() ? link.getSegment(currentSegmentIndex - 1) : link.getSegment(currentSegmentIndex + 1);

            LinkSegmentOrientation linkSegmentOrientation = getLinkAndSegmentOrientation(vehicle.getSegEndX(), vehicle.getSegEndY(), enteringSegment, enteringSegment);

            int stripIndex = vehicle.getNewStripIndex(link.getSegment(currentSegmentIndex), enteringSegment, linkSegmentOrientation.reverseSegment);

            boolean flag = true;
            for (int j = 0; j < vehicle.getNumberOfStrips(); j++) {
                if (!enteringSegment.getStrip(stripIndex + j).hasGapForAddingVehicle(vehicle.getLength())) {
                    flag = false;
                }
            }

            if (flag) {
                vehicle.freeStrips();
                vehicle.decreaseVehicleCountOnSegment();
                vehicle.updateSegmentLeavingData();

                vehicle.setReverseSegment(linkSegmentOrientation.reverseSegment);
                vehicle.segmentChange(enteringSegment.getIndex(), stripIndex);
                vehicle.updateSegmentEnteringData();

                if (linkSegmentOrientation.reverseSegment) {
                    vehicle.setSegStartX(enteringSegment.getEndX());
                    vehicle.setSegStartY(enteringSegment.getEndY());
                    vehicle.setSegEndX(enteringSegment.getStartX());
                    vehicle.setSegEndY(enteringSegment.getStartY());
                } else {
                    vehicle.setSegStartX(enteringSegment.getStartX());
                    vehicle.setSegStartY(enteringSegment.getStartY());
                    vehicle.setSegEndX(enteringSegment.getEndX());
                    vehicle.setSegEndY(enteringSegment.getEndY());
                }
            } else {
                vehicle.setSpeed(0);
            }
        }
    }

    private void moveVehicles() {
        // Record positions for temporal desync attack before any movement
        for (Vehicle vehicle : vehicleList) {
            vehicle.recordPosition();
        }
        for (Vehicle vehicle : vehicleList) {
            System.out.print("");
            if (Parameters.PENALTY_WAIT) {
                if (vehicle.isHasAlreadyCollided()) {
                    if (vehicle.hasPenaltyTimePassed()) {
                        vehicle.removeFromSimulation(false);
                    }
                    continue;
                }
            }
            if (vehicle.isInIntersection()) {
                vehicle.freeStrips();
                if (vehicle.isAtIntersectionEnd()) {
                    moveVehicleAtIntersectionEnd(vehicle);
                } else {
                    vehicle.moveVehicleInIntersection();
                    if (vehicle.isAtIntersectionEnd()) {
                        vehicle.printVehicleDetails();
                        moveVehicleAtIntersectionEnd(vehicle);
                    }
                }
            } else {
                SIGNAL s = getNextSignal(vehicle);
                vehicle.setSignalOnLink(s);
                if (vehicle.isAtSegmentEnd()) {
                    moveVehicleAtSegmentEnd(vehicle);
                    if (vehicle.isInIntersection()) {
                        vehicle.freeStrips();
                        vehicle.printVehicleDetails();
                        vehicle.moveVehicleInIntersection();
                    }
                } else {
                    moveVehicleAtSegmentMiddle(vehicle);

                    if (vehicle.isAtSegmentEnd()) {
                        vehicle.printVehicleDetails();
                        moveVehicleAtSegmentEnd(vehicle);
                        if (vehicle.isInIntersection()) {
                            vehicle.freeStrips();
                            vehicle.printVehicleDetails();
                            vehicle.moveVehicleInIntersection();
                        }
                    }
                }
            }
            vehicle.incrementFuelConsumption();
            vehicle.printVehicleDetails();
            vehicle.addStats();
//            vehicle.printAllLeaders();
        }
        for (Vehicle vehicle : vehicleList) {
            accidentLogAllVehicleAtLast(vehicle);
        }

        for (Vehicle vehicle : vehicleList) {
            accidentCheckAllVehiclesAtLast(vehicle);
        }

        if (Parameters.simulationStep % (60 * Constants.TIME_STEP) == 0) {
            Statistics.flow[(Parameters.simulationStep / (int)(60 * Constants.TIME_STEP)) - 1] = Statistics.flowCount;
            Statistics.flowCount = 0;
        }
    }

    private void controlSignal() {
        for (Node node : intersectionList) {
//            node.adaptiveSignalChange(Parameters.simulationStep);
            node.constantSignalChange(Parameters.simulationStep);
        }
    }

    private SIGNAL getNextSignal(Vehicle vehicle) {
        int demandIndex = vehicle.getDemandIndex();
        int pathIndex = vehicle.getPathIndex();
        int pathLinkIndex = vehicle.getLinkIndexOnPath();
        int lastLinkInPathIndex = demandList.get(demandIndex).getPath(pathIndex).getNumberOfLinks() - 1;
        if (pathLinkIndex == lastLinkInPathIndex) {
            return SIGNAL.GREEN;
        } else {
            int linkIndex = demandList.get(demandIndex).getPath(pathIndex).getLink(pathLinkIndex);
            Node node = vehicle.isReverseLink() ? nodeList.get(vehicle.getLink().getUpNode()) : nodeList.get(vehicle.getLink().getDownNode());
            return node.getSignalOnLink(linkIndex);
        }
    }

    private void updateFlow(double oldDistInSegment, double newDistInSegment, Vehicle vehicle) {
        int linkId = 0;
        int segmentId = 0;
        double sensorDistance = 950;
        boolean direction = true;

        boolean condition1 = vehicle.getLink().getId() == linkId;
        boolean condition2 = vehicle.getSegmentIndex() == segmentId;
        boolean condition3 = vehicle.isReverseLink() == direction;
        boolean condition4 = oldDistInSegment < sensorDistance;
        boolean condition5 = newDistInSegment > sensorDistance;
        if (condition1 && condition2 && condition3 && condition4 && condition5) {
            Statistics.flowCount++;
            if (Parameters.DEBUG_MODE) {
//                System.out.println(Parameters.simulationStep + " ::: " + Statistics.flowCount);
            }
        }
    }


    private void readNetwork() {
        try {
            BufferedReader bufferedReader = new BufferedReader(new InputStreamReader(new FileInputStream(new File("input/link.txt"))));
            String dataLine = bufferedReader.readLine();
            int numLinks = Integer.parseInt(dataLine);
            for (int i = 0; i < numLinks; i++) {
                dataLine = bufferedReader.readLine();
                StringTokenizer stringTokenizer = new StringTokenizer(dataLine, " ");
                int linkId = Integer.parseInt(stringTokenizer.nextToken());
                int nodeId1 = Integer.parseInt(stringTokenizer.nextToken());
                int nodeId2 = Integer.parseInt(stringTokenizer.nextToken());
                int segmentCount = Integer.parseInt(stringTokenizer.nextToken());
                Link link = new Link(i, linkId, nodeId1, nodeId2);
                for (int j = 0; j < segmentCount; j++) {
                    dataLine = bufferedReader.readLine();
                    stringTokenizer = new StringTokenizer(dataLine, " ");
                    int segmentId = Integer.parseInt(stringTokenizer.nextToken());
                    double startX = Double.parseDouble(stringTokenizer.nextToken());
                    double startY = Double.parseDouble(stringTokenizer.nextToken());
                    double endX = Double.parseDouble(stringTokenizer.nextToken());
                    double endY = Double.parseDouble(stringTokenizer.nextToken());
                    double segmentWidth = Double.parseDouble(stringTokenizer.nextToken());

                    boolean firstSegment = (j == 0);
                    boolean lastSegment = (j == segmentCount - 1);

                    Segment segment = new Segment(i, j, segmentId, startX, startY, endX, endY, segmentWidth, lastSegment, firstSegment, linkId);
                    link.addSegment(segment);
                }
                linkList.add(link);
            }
            bufferedReader = new BufferedReader(new InputStreamReader(new FileInputStream(new File("input/node.txt"))));
            dataLine = bufferedReader.readLine();
            int numNodes = Integer.parseInt(dataLine);
            ArrayList<Point2D> boundaryPoints = new ArrayList<>();

            for (int i = 0; i < numNodes; i++) {
                dataLine = bufferedReader.readLine();
                StringTokenizer stringTokenizer = new StringTokenizer(dataLine, " ");
                int nodeId = Integer.parseInt(stringTokenizer.nextToken());
                double centerX = Double.parseDouble(stringTokenizer.nextToken());
                double centerY = Double.parseDouble(stringTokenizer.nextToken());
                if (centerX != 0 || centerY != 0) {
                    boundaryPoints.add(new Point2D(centerX, centerY));
                }
                Node node = new Node(i, nodeId, centerX, centerY);
                while (stringTokenizer.hasMoreTokens()) {
                    node.addLink(getLinkIndex(Integer.parseInt(stringTokenizer.nextToken())));
                }
                if (node.numberOfLinks() > 1) {
                    node.createBundles();
                    intersectionList.add(node);
                }
                nodeList.add(node);
            }

            double left = Double.MAX_VALUE;
            double right = 0;
            double top = Double.MAX_VALUE;
            double down = 0;

            for (Point2D boundaryPoint : boundaryPoints) {

                left = Math.min(left, boundaryPoint.x);
                right = Math.max(right, boundaryPoint.x);

                top = Math.min(top, boundaryPoint.y);
                down = Math.max(down, boundaryPoint.y);
            }
            midPoint = new Point2D((left + right) / 2, (top + down) / 2);

            bufferedReader.close();
        } catch (IOException ex) {
            Logger.getLogger(DhakaSimPanel.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    private int getLinkIndex(int linkId) {
        for (Link link : linkList) {
            if (link.getId() == linkId) {
                return link.getIndex();
            }
        }
        return -1;
    }


    private int getNodeIndex(int nodeId) {
        for (Node node : nodeList) {
            if (node.getId() == nodeId) {
                return node.getIndex();
            }
        }
        return -1;
    }

    @SuppressWarnings("Duplicates")
    private void readDemand() {
        try {
            BufferedReader bufferedReader = new BufferedReader(new InputStreamReader(new FileInputStream(new File("input/demand.txt"))));
            String dataLine = bufferedReader.readLine();
            int numDemands = Integer.parseInt(dataLine);
            for (int i = 0; i < numDemands; i++) {
                dataLine = bufferedReader.readLine();
                StringTokenizer stringTokenizer = new StringTokenizer(dataLine, " ");
                int nodeId1 = Integer.parseInt(stringTokenizer.nextToken());
                int nodeId2 = Integer.parseInt(stringTokenizer.nextToken());
                int demand = Integer.parseInt(stringTokenizer.nextToken());
                
                demandList.add(new Demand(getNodeIndex(nodeId1), getNodeIndex(nodeId2), (int)(demand+30)));
            }
            bufferedReader.close();
        } catch (IOException ex) {
            Logger.getLogger(DhakaSimPanel.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    @SuppressWarnings("Duplicates")
    private void readPath() {
        try {
            BufferedReader bufferedReader = new BufferedReader(new InputStreamReader(new FileInputStream(new File("input/path.txt"))));
            String dataLine = bufferedReader.readLine();
            int numPaths = Integer.parseInt(dataLine);
            for (int i = 0; i < numPaths; i++) {
                dataLine = bufferedReader.readLine();
                StringTokenizer stringTokenizer = new StringTokenizer(dataLine, " ");
                int nodeId1 = Integer.parseInt(stringTokenizer.nextToken());
                int nodeId2 = Integer.parseInt(stringTokenizer.nextToken());
                Path path = new Path(getNodeIndex(nodeId1), getNodeIndex(nodeId2));
                while (stringTokenizer.hasMoreTokens()) {
                    int linkId = Integer.parseInt(stringTokenizer.nextToken());
                    path.addLink(getLinkIndex(linkId));
                }
                pathList.add(path);
            }
            bufferedReader.close();
        } catch (IOException ex) {
            Logger.getLogger(DhakaSimPanel.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    private void addPathToDemand() {
        for (Path path : pathList) {
            for (Demand demand : demandList) {
                if (demand.getSource() == path.getSource() && demand.getDestination() == path.getDestination()) {
                    demand.addPath(path);
                }
            }
        }
    }

    private void totalAccCount(double[] acc) {
        double sum = 0;
        for (double v : acc) {
            sum += v;
        }
//        System.out.println("Total number of accidents: " + sum);
    }
}
