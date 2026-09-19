package thesisfinal;

import java.awt.*;
import java.io.*;
import java.util.logging.Level;
import java.util.logging.Logger;

/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

/**
 * @author USER
 */
public class Pedestrian {
    private int pedestrianId;
    private int startTime;
    boolean toRemove = false;
    private Segment segment;
    private double initPos;
    private double distance;
    private double speed;
    private Strip strip;
    boolean inAccident = false;
    private boolean reverse;
    public int clock = Parameters.simulationStep;
    public int index;

    public Pedestrian(int pedestrianId, Segment seg, int strip, double initpos, double sp) {
        this.pedestrianId = pedestrianId;
        if (strip != 0) {
            reverse = true;
            this.strip = seg.getStrip(seg.numberOfStrips() - 1);
        } else {
            reverse = false;
            this.strip = seg.getStrip(0);
        }
        segment = seg;
        initPos = initpos;
        distance = 0;
        speed = sp;
        this.strip.addPedestrian(this);
        startTime = Parameters.simulationStep;
    }

    void cleanUp() {
        strip.delPedestrian(this);
        toRemove = true;
    }
    boolean isStuck() {
        return Parameters.simulationStep - startTime >= ((segment.getSegmentWidth() / speed) * Constants.TIME_STEP);
    }

    @Override
    public String toString() {
        return "Pedestrian{" +
                "pedestrianId=" + pedestrianId +
                ", initPos=" + initPos +
                ", distance=" + distance +
                ", speed=" + speed +
                ", strip=" + strip.getStripIndex() +
                ", distanceInSegment= " + getDistanceInSegment() +
                '}';
    }

    void printPedestrianDetails() {
        if (Parameters.DEBUG_MODE) {
            if (pedestrianId == 5899 || pedestrianId == -711) {
                String pathname = "debug/p_debug" + pedestrianId + ".txt";
                try (PrintWriter writer = new PrintWriter(new FileOutputStream(new File(pathname), true))) {
                    writer.println("Sim step: " + Parameters.simulationStep);
                    writer.println("Pedestrian ID: " + pedestrianId);
                    writer.println("Init Pos: " + initPos);
                    writer.printf("Speed: %.2f\n", speed);
                    writer.printf("Distance in Segment: %.2f\n", getDistanceInSegment());
                    writer.println("Strip Index: " + strip.getStripIndex());
                    writer.println();
                } catch (FileNotFoundException e) {
                    e.printStackTrace();
                }
            }

        }
    }
    public int getPedestrianId() {
        return pedestrianId;
    }

    private double getDistance() {
        return distance;
    }

    public double getSpeed() {
        return speed;
    }

    double getInitPos() {
        return initPos;
    }

    boolean getReverseSegment() {
        return strip.getStripIndex() > segment.middleLowStripIndex;
    }

    double getDistanceInSegment() {
        int stripIndex = strip.getStripIndex();
        boolean reverseSegment = getReverseSegment();

        if (reverseSegment) {
            return segment.getLength() - initPos;
        } else {
            return initPos;
        }
    }

    Segment getSegment() {
        return segment;
    }

    // TODO ????
    boolean moveLengthWise() {
        boolean reverseSegment = getReverseSegment();
        if (reverseSegment) {
            if (strip.hasGapForMoveAlongPositive(this)) {
                this.initPos += this.speed;
                return true;
            }
        } else {
            if (strip.hasGapForMoveAlongNegative(this)) {
                this.initPos -= this.speed;
                return true;
            }
        }
        return false;
    }

    boolean moveForward() {
        if (!reverse) {
            int x;
            if (distance + speed < Parameters.footpathStripWidth) {
                distance += speed;
                return true;
            }
            x = 1 + (int) ((distance - Parameters.footpathStripWidth + speed) / Parameters.stripWidth);
            if (x < segment.numberOfStrips()) {
                if (segment.getStrip(x).hasGapForPedestrian(this)) {
                    distance = distance + speed;
                    strip.delPedestrian(this);
                    setStrip(segment.getStrip(x));
                    strip.addPedestrian(this);
                    return true;
                }
                return false;
            } else {
                distance = distance + speed;
                strip.delPedestrian(this);
                return true;
            }
        } else {
            int x;
            double w = 1 * Parameters.footpathStripWidth + (segment.numberOfStrips() - 1) * Parameters.stripWidth;
            x = 1 + (int) ((w - distance - speed - Parameters.footpathStripWidth) / Parameters.stripWidth);
            if (x > 1) {
                if (segment.getStrip(x).hasGapForPedestrian(this)) {
                    distance = distance + speed;
                    strip.delPedestrian(this);
                    setStrip(segment.getStrip(x));
                    strip.addPedestrian(this);
                    return true;
                }
                return false;
            } else {
                distance = distance + speed;
                strip.delPedestrian(this);
                return true;
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
        //System.out.println(distance + " " + segment.getSegWidth() + " " + strip.getStripIndex());
        return distance >= segment.getSegmentWidth();
    }

    void drawMobilePedestrian(BufferedWriter traceWriter, Graphics2D g, double stripPixelCount, double mpRatio, double fpStripPixelCount) {
        if (!reverse) {
            Segment seg = getSegment();
            double segmentLength = seg.getLength();
            double length = 1;
            //Using internally section or ratio formula,it finds the coordinates along which vehicles are
            double xp = (getInitPos() * seg.getEndX() + (segmentLength - getInitPos()) * seg.getStartX()) / segmentLength * mpRatio;
            double yp = (getInitPos() * seg.getEndY() + (segmentLength - getInitPos()) * seg.getStartY()) / segmentLength * mpRatio;
            double xq = ((getInitPos() + length) * seg.getEndX() + (segmentLength - (getInitPos() + length)) * seg.getStartX()) / segmentLength * mpRatio;
            double yq = ((getInitPos() + length) * seg.getEndY() + (segmentLength - (getInitPos() + length)) * seg.getStartY()) / segmentLength * mpRatio;
            int x1 = (int) Math.round(Utilities.returnX3(xp, yp, xq, yq, (getDistance() / Parameters.footpathStripWidth) * fpStripPixelCount)); //obj.getDistance()*mpRatio
            int y1 = (int) Math.round(Utilities.returnY3(xp, yp, xq, yq, (getDistance() / Parameters.footpathStripWidth) * fpStripPixelCount));
            if (inAccident) {
                g.setColor(Color.red);
                g.fillOval(x1, y1,(int)(1.5 * mpRatio), (int)(1.5 * mpRatio));
            } else {
                g.setColor(Constants.pedestrianColor);
                g.fillOval(x1, y1, (int)(0.4 * mpRatio), (int)(0.4 * mpRatio));
            }

            if (Parameters.DEBUG_MODE) {
                Font font = new Font("Serif", Font.PLAIN, 64);
                g.setFont(font);
                g.drawString(Integer.toString(pedestrianId), x1, y1);
            }
            try {
                traceWriter.write(x1 + " " + y1 + " " + inAccident);
                traceWriter.newLine();
            } catch (IOException ex) {
                Logger.getLogger(Pedestrian.class.getName()).log(Level.SEVERE, null, ex);
            }

        } else {
            Segment seg = getSegment();
            double segmentLength = seg.getLength();
            double length = 1;
            //Using internally section or ratio formula,it finds the coordinates along which vehicles are
            double xp = (getInitPos() * seg.getEndX() + (segmentLength - getInitPos()) * seg.getStartX()) / segmentLength * mpRatio;
            double yp = (getInitPos() * seg.getEndY() + (segmentLength - getInitPos()) * seg.getStartY()) / segmentLength * mpRatio;
            double xq = ((getInitPos() + length) * seg.getEndX() + (segmentLength - (getInitPos() + length)) * seg.getStartX()) / segmentLength * mpRatio;
            double yq = ((getInitPos() + length) * seg.getEndY() + (segmentLength - (getInitPos() + length)) * seg.getStartY()) / segmentLength * mpRatio;
            double w = 1 * Parameters.footpathStripWidth + (segment.numberOfStrips() - 1) * Parameters.stripWidth;
            int wi = (int) (w - getDistance());
            int x1 = (int) Math.round(Utilities.returnX3(xp, yp, xq, yq, (wi / Parameters.footpathStripWidth) * fpStripPixelCount)); //obj.getDistance()*mpRatio
            int y1 = (int) Math.round(Utilities.returnY3(xp, yp, xq, yq, (wi / Parameters.footpathStripWidth) * fpStripPixelCount));
            if (inAccident) {
                g.setColor(Color.red);
                g.fillOval(x1, y1,(int)(1.2 * mpRatio), (int)(1.2 * mpRatio));
            } else {
                g.setColor(Constants.pedestrianColor);
                g.fillOval(x1, y1, (int)(0.4 * mpRatio), (int)(0.4 * mpRatio));
            }
            if (Parameters.DEBUG_MODE) {
                Font font = new Font("Serif", Font.PLAIN, 64);
                g.setFont(font);
                g.drawString(Integer.toString(pedestrianId), x1, y1);
            }
            try {
                traceWriter.write(x1 + " " + y1 + " " + inAccident);
                traceWriter.newLine();
            } catch (IOException ex) {
                Logger.getLogger(Pedestrian.class.getName()).log(Level.SEVERE, null, ex);
            }
        }
    }

    void printObject() {
        System.out.println(index + " " + initPos + " " + distance + " " + speed);
    }

    boolean isToRemove() {
        return toRemove;
    }

    boolean isInAccident() {
        return inAccident;
    }

    void setToRemove(boolean b) {
        toRemove = b;
    }
}
