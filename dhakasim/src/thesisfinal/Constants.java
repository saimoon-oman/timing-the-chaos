package thesisfinal;

import static thesisfinal.Parameters.ACROSS_PEDESTRIAN_PER_HOUR;
import static thesisfinal.Parameters.ALONG_PEDESTRIAN_PER_HOUR;

import java.awt.*;
import org.apache.commons.math3.distribution.PoissonDistribution;

public interface Constants {
    int TYPES_OF_CARS = 13;
	int PEDESTRIANS_ALONG_THE_ROAD_TYPE = 12;
    Color pedestrianColor = Color.BLACK;
    Color roadBorderColor = Color.BLACK;
    Color backgroundColor = Color.WHITE;// new Color(105, 105, 105);//Color.DARK_GRAY;
    double DEFAULT_SCALE = 5;
    double TIME_STEP = 1.0;
    double THRESHOLD_DISTANCE = 0.5;
    boolean PRINT_RESULT = true;

    int MAX_NUMBER_OF_OBJECTS = Integer.MAX_VALUE;
	int MAX_NUMBER_OF_VEHICLES = Integer.MAX_VALUE;

	double TOTAL_NETWORK_ROAD_LENGTH = 3.83;



	Color ROAD_BORDER_COLOR = Color.black;
	Color BACKGROUND_COLOR = new Color(240, 240, 240);



	
	double PEDESTRIAN_SIZE = 0.7; //Unit: meter	
	double p = (double)(ACROSS_PEDESTRIAN_PER_HOUR*1.0)/3600;	
	PoissonDistribution road_crossing_ped_poisson = new PoissonDistribution(ACROSS_PEDESTRIAN_PER_HOUR/3600);
	PoissonDistribution road_along_ped_poisson = new PoissonDistribution(ALONG_PEDESTRIAN_PER_HOUR/3600);

	


	Color STANDING_PEDESTRIAN_COLOR = new Color(216, 150, 0);
	double STANDING_PEDESTRIAN_LENGTH = PEDESTRIAN_SIZE; //Unit: meter
	double STANDING_PEDESTRIAN_WIDTH = PEDESTRIAN_SIZE; //Unit: meter
	int STANDING_PEDESTRIAN_TIME_LIMIT_FACTOR = 50;
	double STANDING_PEDESTRIAN_MIN_BLOCKAGE = 0.21; //Unit: meter
	double STANDING_PEDESTRIAN_MAX_BLOCKAGE = 6.00; //Unit: meter
	NormalDistribution[] STANDING_PEDESTRIAN_DISTRIBUTIONS = new NormalDistribution[] {
			new NormalDistribution(0.4635,0.825,0.40),
			new NormalDistribution(0.3250,1.925,0.45),
			new NormalDistribution(0.1350,2.900,0.42),
			new NormalDistribution(0.0640,3.710,0.35),
			new NormalDistribution(0.0070,5.750,0.40)
	};
	int AVG_NUMBER_OF_STANDING_PEDESTRIANS = (int)(Math.ceil(15.61*TOTAL_NETWORK_ROAD_LENGTH));


	Color PARKED_CAR_COLOR = new Color(216, 150, 0);
	double PARKED_CAR_LENGTH = 4.5; //Unit: meter
	double PARKED_CAR_WIDTH = 1.7; //Unit: meter
	int PARKED_CAR_TIME_LIMIT_FACTOR = STANDING_PEDESTRIAN_TIME_LIMIT_FACTOR * 10;
	double PARKED_CAR_MIN_BLOCKAGE = 0.70; //Unit: meter
	double PARKED_CAR_MAX_BLOCKAGE = 6.00; //Unit: meter
	NormalDistribution[] PARKED_CAR_DISTRIBUTIONS = new NormalDistribution[] {
			new NormalDistribution(0.050,1.12,0.390),
			new NormalDistribution(0.515,2.07,0.322),
			new NormalDistribution(0.200,2.73,0.310),
			new NormalDistribution(0.172,3.64,0.334),
			new NormalDistribution(0.042,4.48,0.300),
			new NormalDistribution(0.010,5.92,0.310)
	};
	int AVG_NUMBER_OF_PARKED_CARS = (int)(Math.ceil(19.77*TOTAL_NETWORK_ROAD_LENGTH));



	Color PARKED_RICKSHAW_COLOR = new Color(216, 150, 0);
	double PARKED_RICKSHAW_LENGTH = 3; //Unit: meter
	double PARKED_RICKSHAW_WIDTH = 1; //Unit: meter
	int PARKED_RICKSHAW_TIME_LIMIT_FACTOR = PARKED_CAR_TIME_LIMIT_FACTOR/5;
	double PARKED_RICKSHAW_MIN_BLOCKAGE = 0.56; //Unit: meter
	double PARKED_RICKSHAW_MAX_BLOCKAGE = 6.00; //Unit: meter
	NormalDistribution[] PARKED_RICKSHAW_DISTRIBUTIONS = new NormalDistribution[] {
			new NormalDistribution(0.582,1.57,0.500),
			new NormalDistribution(0.258,2.76,0.440),
			new NormalDistribution(0.085,3.67,0.320),
			new NormalDistribution(0.046,4.65,0.380),
			new NormalDistribution(0.009,5.88,0.380)
	};
	int AVG_NUMBER_OF_PARKED_RICKSHAWS = (int)(Math.ceil(15.61*TOTAL_NETWORK_ROAD_LENGTH));



	Color PARKED_CNG_COLOR = new Color(216, 150, 0);
	double PARKED_CNG_LENGTH = 2.6; //Unit: meter
	double PARKED_CNG_WIDTH = 1.3; //Unit: meter   
	int PARKED_CNG_TIME_LIMIT_FACTOR = PARKED_CAR_TIME_LIMIT_FACTOR/5;
	double PARKED_CNG_MIN_BLOCKAGE = 0.95; //Unit: meter
	double PARKED_CNG_MAX_BLOCKAGE = 4.67; //Unit: meter
	NormalDistribution[] PARKED_CNG_DISTRIBUTIONS = new NormalDistribution[] {
			new NormalDistribution(0.119,1.13,0.29),
			new NormalDistribution(0.730,2.30,0.55),
			new NormalDistribution(0.065,3.81,0.27),
			new NormalDistribution(0.015,4.65,0.32)
	};
	int AVG_NUMBER_OF_PARKED_CNGS = (int)(Math.ceil(4.67*TOTAL_NETWORK_ROAD_LENGTH));
	
	
	double WALKING_PEDESTRIAN_MIN_BLOCKAGE = 0.50; //Unit: meter
	double WALKING_PEDESTRIAN_MAX_BLOCKAGE = 7.75; //Unit: meter
	NormalDistribution[] WALKING_PEDESTRIAN_DISTRIBUTIONS = new NormalDistribution[] {
			new NormalDistribution(0.529,1.60,0.531),
			new NormalDistribution(0.471,3.38,1.388)
	};
	

}