package thesisfinal;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.PrintWriter;
import java.util.ArrayList;

public class Statistics {
    static double[] avgSpeedOfVehicle;
    static int[] noOfVehicles;
    static int[] waitingTime;
    static int[] totalTravelTime;
    static int[][] tripTime;
    static double[][] noOfVehiclesCompletingTrip;
    static double[][] totalFuelConsumption;
    static int noOfCollisions;
    static int noOfAccidents;
    static double[][] noCollisionsPerDemand;
    static double[][] noAccidentsPerDemand;
    static double[] flow;
    static int flowCount;
    static double[] noOfGeneratedVehicles;

    // Gap error statistics for temporal desync attacks
    static ArrayList<double[]> gapErrors;
    static double totalGapError;
    static long totalGapErrorCount;

    static ArrayList<VehicleStats> vehicleStats;

    Statistics(int demandSize) {
        avgSpeedOfVehicle = new double[Constants.TYPES_OF_CARS];
        noOfVehicles = new int[Constants.TYPES_OF_CARS];
        noOfGeneratedVehicles = new double[Constants.TYPES_OF_CARS];
        waitingTime = new int[Constants.TYPES_OF_CARS];
        totalTravelTime = new int[Constants.TYPES_OF_CARS];
        totalFuelConsumption = new double[demandSize][Constants.TYPES_OF_CARS];

        noOfVehiclesCompletingTrip = new double[demandSize][Constants.TYPES_OF_CARS];
        noCollisionsPerDemand = new double[demandSize][Constants.TYPES_OF_CARS];
        noAccidentsPerDemand = new double[demandSize][Constants.TYPES_OF_CARS];
        tripTime = new int[demandSize][Constants.TYPES_OF_CARS];
        vehicleStats = new ArrayList<>();
        noOfCollisions = 0;
        noOfAccidents = 0;

        flow = new double[Parameters.simulationEndTime / (int)(60 * Constants.TIME_STEP)];
        flowCount = 0;

        gapErrors = new ArrayList<>();
        totalGapError = 0;
        totalGapErrorCount = 0;
    }

    static void addGapError(Vehicle leader, Vehicle follower, double error, double perceivedGap, double trueGap, int step) {
        gapErrors.add(new double[]{step, leader.getVehicleId(), follower.getVehicleId(), error, perceivedGap, trueGap, leader.getSpeed(), follower.getSpeed()});
        totalGapError += Math.abs(error);
        totalGapErrorCount++;
    }

    static void printGapErrorStatistics() {
        try (PrintWriter writer = new PrintWriter(new FileOutputStream("statistics/gap_errors.csv", false))) {
            writer.println("simStep,leaderId,followerId,gapError,perceivedGap,trueGap,leaderSpeed,followerSpeed");
            for (double[] entry : gapErrors) {
                writer.printf("%.0f,%.0f,%.0f,%.4f,%.4f,%.4f,%.2f,%.2f%n", entry[0], entry[1], entry[2], entry[3], entry[4], entry[5], entry[6], entry[7]);
            }
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
        System.out.println("Gap error count: " + totalGapErrorCount);
        System.out.println("Mean absolute gap error: " + (totalGapErrorCount > 0 ? totalGapError / totalGapErrorCount : 0));
    }

    static void printSafetyStatistics() {
        System.out.println("Collision count: " + noOfCollisions);
        System.out.println("Accident count: " + noOfAccidents);
    }

    static void saveVehicleStat(VehicleStats stats) {
        int[] vehicleIds = {1, 2, 3};

        for (int id : vehicleIds) {
            if (id == stats.getVehicleId()) {
                vehicleStats.add(stats);
            }
        }
    }
}
