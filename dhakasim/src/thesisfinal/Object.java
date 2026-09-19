package thesisfinal;

import java.awt.*;
import java.io.BufferedWriter;
import java.io.IOException;
import java.util.Random;
import java.util.logging.Level;
import java.util.logging.Logger;

import static thesisfinal.Constants.*;
import static thesisfinal.Parameters.*;

public class Object{

	/**
	 * Current object types are:
	 * <ul>
	 *     <li>0 for road-crossing pedestrian</li>
	 *     <li>1 for standing pedestrian</li>
	 *     <li>2 for parked car</li>
	 *     <li>3 for parked rickshaw</li>
	 *     <li>4 for parked CNG</li>
	 * </ul>
	 */
    private double objectId;
	public int objectType;
	private Segment segment;
	private Strip strip;
	/**
	 * Equivalent to {@link Vehicle#getDistanceInSegment}.
	 */
	private double initPos;
	
	private double objectLength;
	private double objectWidth;
	private double distanceFromFootpath;
	private double distanceAlongWidth;	
	private boolean reverseDirection;
	private double randomDouble;
	boolean inAccident = false;
	boolean toRemove = false;
	public int index;
	private double parking_time; // we estimate the parking time of the object from a uniform distribution
	private double parking_start_time; // the time when the object was parked (in case of standing pedestrian, the time when the pedestrian stood in that position blocking the road)

	public Object (int objectId, int objectType, int parking_start_time, Link link, Segment segment, int segmentId, double initPos, boolean reverseDirection, double distanceFromFootpath) {

		//Initialize parameters
		this.objectType = objectType;
		this.segment = segment;
		this.initPos = initPos;		
		this.reverseDirection = reverseDirection;
		this.parking_start_time = parking_start_time;		

		Random random = new Random();
		randomDouble = random.nextDouble();



		//Initialize other fields
		if (objectType==1) {
			objectLength = STANDING_PEDESTRIAN_LENGTH;
			objectWidth = STANDING_PEDESTRIAN_WIDTH;
			parking_time= new Random().nextDouble(10,60);

		}
		else if (objectType==2) {
			objectLength = PARKED_CAR_LENGTH;
			objectWidth = PARKED_CAR_WIDTH;
			parking_time= new Random().nextDouble(100,500);

		}
		else if (objectType==3) {
			objectLength = PARKED_RICKSHAW_LENGTH;
			objectWidth = PARKED_RICKSHAW_WIDTH;
			parking_time= new Random().nextDouble(30,150);
		}
		else if (objectType==4) {
			objectLength = PARKED_CNG_LENGTH;
			objectWidth = PARKED_CNG_WIDTH;   
			parking_time= new Random().nextDouble(60,300);      
		}

        // distanceFromFootpath = Utilities.getRandomFromMultipleGaussianDistributionOfObjectsBlockage(objectType) - objectWidth;
		this.distanceFromFootpath = distanceFromFootpath;



		distanceAlongWidth = distanceFromFootpath + footpathStripWidth;

		if (reverseDirection) {
			strip = segment.getStrip(segment.numberOfStrips() - 1);
		} else {
			strip = segment.getStrip(0);
		}

		occupyStrips();
	}

	
	public double getDistanceInSegment() {
		return initPos;		
	}

	protected double getDistanceAlongWidth() {
		return distanceAlongWidth;
	}

	protected double getDistanceFromFootpath() {
		return distanceFromFootpath;
	}

	double getInitPos() {
		return initPos;
	}

	double getInitPosOfStartingSide() {
		if (objectType==0 || objectType==1) {
			return initPos - objectLength /2;
		}
		else return initPos;
	}

	double getObjectLength () {
		return objectLength;
	}

	Segment getSegment() {
		return segment;
	}

	double getParkingStartTime()
	{
		return parking_start_time;
	}

	double getParkingTime()
	{
		return parking_time;
	}
	

	private void occupyStrips() {
		double occupiedWidth = distanceFromFootpath + objectWidth;
		occupyStripsForRectangularObject(occupiedWidth);
	}

	private void occupyStripsForRectangularObject(double width) {
		
		int numberOfOccupiedStrips;
		if (width%stripWidth==0) {
			numberOfOccupiedStrips = (int) (width/stripWidth);
		}
		else {
			numberOfOccupiedStrips = (int) (width/stripWidth) + 1;
		}
		if (!reverseDirection) {
			for (int i=0; i<=numberOfOccupiedStrips; i++) { //Started from 0, as we want to occupy the footpath strip as well.
				segment.getStrip(i).addObject(this);
			}
		} else {
			for (int i=0; i<=numberOfOccupiedStrips; i++) { //Started from 0, as we want to occupy the footpath strip as well.
				segment.getStrip(segment.numberOfStrips()-1 - i).addObject(this);
			}
		}
	}

	public Strip getStrip() {
		return strip;
	}

	public void setSegment(Segment segment) {
		this.segment = segment;
	}

	private void setStrip(Strip strip) {
		this.strip = strip;
	}

	boolean hasCrossedRoad() {
		return distanceAlongWidth >= segment.getSegmentWidth();
	}

	void drawObject (BufferedWriter traceWriter, Graphics2D g, double stripPixelCount, double mpRatio, double fpStripPixelCount) {

		if (objectType==1) {
			drawRectangularObject(g, STANDING_PEDESTRIAN_LENGTH, STANDING_PEDESTRIAN_WIDTH, STANDING_PEDESTRIAN_COLOR);
		}
		else if (objectType==2) {
			drawRectangularObject(g, PARKED_CAR_LENGTH, PARKED_CAR_WIDTH, PARKED_CAR_COLOR);
		}
		else if (objectType==3) {
			drawRectangularObject(g, PARKED_RICKSHAW_LENGTH, PARKED_RICKSHAW_WIDTH, PARKED_RICKSHAW_COLOR);
		}
		else if (objectType==4) {
			drawRectangularObject(g, PARKED_CNG_LENGTH, PARKED_CNG_WIDTH, PARKED_CNG_COLOR);
		}
		
	}

	private void drawRectangularObject(Graphics2D g, double length, double width, Color color) {
		int x1,y1,x2,y2,x3,y3,x4,y4;
		int[] xs;
		int[] ys;
		Segment segment = getSegment();
		double segmentLength = segment.getLength();
		double distanceInSegment = initPos;

		double xp = (distanceInSegment * segment.getEndX() + (segmentLength - distanceInSegment) * segment.getStartX()) / segmentLength * pixelPerMeter;
		double yp = (distanceInSegment * segment.getEndY() + (segmentLength - distanceInSegment) * segment.getStartY()) / segmentLength * pixelPerMeter;
		double xq = ((distanceInSegment + length) * segment.getEndX() + (segmentLength - (distanceInSegment + length)) * segment.getStartX()) / segmentLength * pixelPerMeter;
		double yq = ((distanceInSegment + length) * segment.getEndY() + (segmentLength - (distanceInSegment + length)) * segment.getStartY()) / segmentLength * pixelPerMeter;

		double w;
		int objectWidthInPixel = (int) Math.round(width * pixelPerMeter);
		if (!reverseDirection) {
			w = (distanceAlongWidth) * pixelPerMeter;
			x1 = (int) Math.round(Utilities.returnX3(xp, yp, xq, yq, w));
			y1 = (int) Math.round(Utilities.returnY3(xp, yp, xq, yq, w));
			x2 = (int) Math.round(Utilities.returnX4(xp, yp, xq, yq, w));
			y2 = (int) Math.round(Utilities.returnY4(xp, yp, xq, yq, w));
			x3 = (int) Math.round(Utilities.returnX3(xp, yp, xq, yq, w + objectWidthInPixel));
			y3 = (int) Math.round(Utilities.returnY3(xp, yp, xq, yq, w + objectWidthInPixel));
			x4 = (int) Math.round(Utilities.returnX4(xp, yp, xq, yq, w + objectWidthInPixel));
			y4 = (int) Math.round(Utilities.returnY4(xp, yp, xq, yq, w + objectWidthInPixel));
		}
		else {
			w = (segment.getSegmentWidth()-distanceAlongWidth-width) * pixelPerMeter;
			x1 = (int) Math.round(Utilities.returnX3(xp, yp, xq, yq, w));
			y1 = (int) Math.round(Utilities.returnY3(xp, yp, xq, yq, w));
			x2 = (int) Math.round(Utilities.returnX4(xp, yp, xq, yq, w));
			y2 = (int) Math.round(Utilities.returnY4(xp, yp, xq, yq, w));
			x3 = (int) Math.round(Utilities.returnX3(xp, yp, xq, yq, w + objectWidthInPixel));
			y3 = (int) Math.round(Utilities.returnY3(xp, yp, xq, yq, w + objectWidthInPixel));
			x4 = (int) Math.round(Utilities.returnX4(xp, yp, xq, yq, w + objectWidthInPixel));
			y4 = (int) Math.round(Utilities.returnY4(xp, yp, xq, yq, w + objectWidthInPixel));
		}
		xs = new int[]{x1, x2, x4, x3};
		ys = new int[]{y1, y2, y4, y3};
		g.setColor(color);
		g.fillPolygon(xs, ys, 4);
	}

	

	public boolean isToRemove() {
		
		return toRemove;
	}

	boolean isInAccident() {
		return inAccident;
	}

	public boolean isReverseSegment () {
		return reverseDirection;
	}

	public void setToRemove(boolean b) {
		toRemove = b;
	}
	
	


}
