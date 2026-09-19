package thesisfinal;

import javax.swing.*;
import java.awt.*;
import java.awt.event.*;
import java.awt.geom.AffineTransform;
import java.io.*;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Random;
import java.util.StringTokenizer;
import java.util.logging.Level;
import java.util.logging.Logger;

import static java.lang.Math.round;
import static thesisfinal.Utilities.*;

public class DhakaSimPanel extends JPanel implements ActionListener, MouseListener, MouseMotionListener, MouseWheelListener {
    private LinkedList<Pedestrian> pedestrians;
    private LinkedList<Vehicle> vehicleList;
    private LinkedList<Object> objectList;

    private ArrayList<Node> nodeList;
    private ArrayList<Link> linkList;
    private Point2D midPoint;
    private double translateX;
    private double translateY;
    private int referenceX = -999999999;
    private int referenceY = -999999999;
    private double scale = 0.10 + Constants.DEFAULT_SCALE * 0.05;

    private LineNumberReader traceReader;
    private BufferedWriter traceWriter;
    private final JFrame jFrame;
    private final Timer timer;

    private final boolean drawRoads = true;
    private final boolean drawTrajectories = false;
    private final Random random = Parameters.random;
    private int vehicleId = 0;

    private Processor processor;

    public DhakaSimPanel(JFrame jFrame) {
        this.jFrame = jFrame;

        try {
            if (Parameters.TRACE_MODE) {
                traceReader = new LineNumberReader(new FileReader("trace.txt"));
                StringTokenizer tokenizer = new StringTokenizer(traceReader.readLine(), " ");
                Parameters.simulationSpeed = Integer.parseInt(tokenizer.nextToken());
                Parameters.simulationEndTime = Integer.parseInt(tokenizer.nextToken());
                Parameters.acrossPedestrianMode = Boolean.parseBoolean(tokenizer.nextToken());
            } else {
                traceWriter = new BufferedWriter(new FileWriter("trace.txt", false));
                traceWriter.write(Parameters.simulationSpeed + " " + Parameters.simulationEndTime + " " + Parameters.acrossPedestrianMode);
                traceWriter.newLine();
                traceWriter.flush();
            }
        } catch (IOException fe) {
            fe.printStackTrace();
        }

        this.addMouseListener(this);
        this.addMouseMotionListener(this);

        processor = new Processor();

        midPoint = processor.getMidPoint();
        pedestrians = processor.getPedestrians();
        vehicleList = processor.getVehicleList();
        objectList = processor.getObjectList();
        nodeList = processor.getNodeList();
        linkList = processor.getLinkList();

        if (Parameters.CENTERED_VIEW) {
            translateX = -midPoint.x * Parameters.pixelPerMeter;
            translateY = -midPoint.y * Parameters.pixelPerMeter;
        } else {
            translateX = Parameters.DEFAULT_TRANSLATE_X;
            translateY = Parameters.DEFAULT_TRANSLATE_Y;
        }
        timer = new Timer(Parameters.simulationSpeed, this);
        timer.start();
    }

    public void setScale(double scale) {
        this.scale = scale;
    }

    @Override
    public void paintComponent(Graphics g) {
        Graphics2D g2d = (Graphics2D) g;
        super.paintComponent(g2d);
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        g2d.setColor(Constants.backgroundColor);
        g2d.fillRect(0, 0, getWidth(), getHeight());
        AffineTransform affineTransform = g2d.getTransform();
        affineTransform.translate(getWidth() / 2.0, getHeight() / 2.0);
        affineTransform.scale(scale, scale);
        affineTransform.translate(-getWidth() / 2.0, -getHeight() / 2.0);
        affineTransform.translate(translateX, translateY);
        g2d.setTransform(affineTransform);

        if (drawRoads) {
            drawRoadNetwork(g2d);
        }

        if (drawTrajectories) {
            drawAllTrajectories(g2d);
        }

        if (Parameters.TRACE_MODE) {
            Utilities.drawTrace(traceReader, g2d);
        } else {
            if (Parameters.acrossPedestrianMode) {
                try {
                    traceWriter.write("Current Pedestrians");
                    traceWriter.newLine();
                } catch (IOException ex) {
                    Logger.getLogger(DhakaSimPanel.class.getName()).log(Level.SEVERE, null, ex);
                }
                for (Pedestrian pedestrian : pedestrians) {
                    pedestrian.drawMobilePedestrian(traceWriter, g2d, Parameters.pixelPerStrip, Parameters.pixelPerMeter, Parameters.pixelPerFootpathStrip);
                }
            }
            try {
                traceWriter.write("Current Vehicles");
                traceWriter.newLine();
            } catch (IOException ex) {
                Logger.getLogger(DhakaSimPanel.class.getName()).log(Level.SEVERE, null, ex);
            }
            for (Vehicle vehicle : vehicleList) {
                vehicle.drawVehicle(traceWriter, g2d, Parameters.pixelPerStrip, Parameters.pixelPerMeter, Parameters.pixelPerFootpathStrip);
            }

            try {
                traceWriter.write("Current Objects");
                traceWriter.newLine();
            } catch (IOException ex) {
                Logger.getLogger(DhakaSimPanel.class.getName()).log(Level.SEVERE, null, ex);
            }
            for (Object object : objectList) {
                object.drawObject(traceWriter, g2d, Parameters.pixelPerStrip, Parameters.pixelPerMeter, Parameters.pixelPerFootpathStrip);
            }

            try {
                traceWriter.write("End Step");
                traceWriter.newLine();
                traceWriter.flush();
            } catch (IOException ex) {
                Logger.getLogger(DhakaSimPanel.class.getName()).log(Level.SEVERE, null, ex);
            }
        }
    }

    private void drawAllTrajectories(Graphics2D graphics2D) {
        // TODO
        if (Parameters.simulationEndTime <= Parameters.simulationStep) {
            for (VehicleStats vs : Statistics.vehicleStats) {
                Color[] colors = {Color.GREEN, Color.RED, Color.CYAN, Color.MAGENTA, Color.BLUE};
                drawTrajectory(graphics2D, vs.getTrajectory(), colors[vs.getVehicleId()]);

            }
        }

    }
    private void drawTrajectory(Graphics2D g2d, Point2D[] points, Color color) {
        g2d.setColor(color);
        g2d.setStroke(new BasicStroke(10));
        for (int i = 0; i < points.length - 1; i++) {
            if (points[i] == null || points[i + 1] == null)
                continue;
            g2d.drawLine((int)points[i].x, (int)points[i].y, (int)points[i+1].x, (int)points[i+1].y);
        }
    }
    private void drawNodeId(Graphics2D g2d, Node node) {
        Font font = new Font("Serif", Font.BOLD, 256);
        g2d.setFont(font);
        Color color_bu = g2d.getColor();
        g2d.setColor(Color.BLACK);
        g2d.drawString(Integer.toString(node.getId()), (int) (node.x * Parameters.pixelPerMeter), (int) (node.y * Parameters.pixelPerMeter));
        g2d.setColor(color_bu);
    }

    @SuppressWarnings("Duplicates")
    private void drawRoadNetwork(Graphics2D g2d) {
        g2d.setColor(Color.WHITE);
        for (Link link : linkList) {
            link.draw(g2d);
        }
        ArrayList<double[]> lineList = new ArrayList<>();
        for (Node node : nodeList) {
            g2d.setColor(Color.BLACK);
            drawNodeId(g2d, node);
            for (int j = 0; j < node.numberOfLinks(); j++) {
                Link link = linkList.get(node.getLink(j));
                double x1, y1, x2, y2, x3, y3, x4, y4;
                if (link.getUpNode() == node.getId()) {
                    x1 = link.getFirstSegment().getStartX() * Parameters.pixelPerMeter;
                    y1 = link.getFirstSegment().getStartY() * Parameters.pixelPerMeter;
                    x2 = link.getFirstSegment().getEndX() * Parameters.pixelPerMeter;
                    y2 = link.getFirstSegment().getEndY() * Parameters.pixelPerMeter;

                    //double width = 2 * pixelPerFootpathStrip + (link.getSegment(0).numberOfStrips() - 2) * pixelPerStrip;
                    double width = link.getFirstSegment().getSegmentWidth() * Parameters.pixelPerMeter;

                    x3 = returnX3(x1, y1, x2, y2, width);
                    y3 = returnY3(x1, y1, x2, y2, width);
                    x4 = returnX4(x1, y1, x2, y2, width);
                    y4 = returnY4(x1, y1, x2, y2, width);
                } else {
                    x1 = link.getLastSegment().getStartX() * Parameters.pixelPerMeter;
                    y1 = link.getLastSegment().getStartY() * Parameters.pixelPerMeter;
                    x2 = link.getLastSegment().getEndX() * Parameters.pixelPerMeter;
                    y2 = link.getLastSegment().getEndY() * Parameters.pixelPerMeter;

//                    double w = 2 * pixelPerFootpathStrip + (link.getSegment(link.getNumberOfSegments() - 1).numberOfStrips() - 2) * pixelPerStrip;
                    double w = link.getLastSegment().getSegmentWidth() * Parameters.pixelPerMeter;

                    x3 = returnX3(x1, y1, x2, y2, w);
                    y3 = returnY3(x1, y1, x2, y2, w);
                    x4 = returnX4(x1, y1, x2, y2, w);
                    y4 = returnY4(x1, y1, x2, y2, w);

                    x1 = x2;
                    y1 = y2;
                    x3 = x4;
                    y3 = y4;
                }
                for (int k = 0; k < node.numberOfLinks(); k++) {
                    if (j != k) {
                        Link linkPrime = linkList.get(node.getLink(k));
                        double x1Prime, y1Prime, x2Prime, y2Prime, x3Prime, y3Prime, x4Prime, y4Prime;
                        if (linkPrime.getUpNode() == node.getId()) {
                            x1Prime = linkPrime.getFirstSegment().getStartX() * Parameters.pixelPerMeter;
                            y1Prime = linkPrime.getFirstSegment().getStartY() * Parameters.pixelPerMeter;
                            x2Prime = linkPrime.getFirstSegment().getEndX() * Parameters.pixelPerMeter;
                            y2Prime = linkPrime.getFirstSegment().getEndY() * Parameters.pixelPerMeter;

                            //double w = 2 * pixelPerFootpathStrip + (linkPrime.getSegment(0).numberOfStrips() - 2) * pixelPerStrip;
                            double w = linkPrime.getFirstSegment().getSegmentWidth() * Parameters.pixelPerMeter;

                            x3Prime = returnX3(x1Prime, y1Prime, x2Prime, y2Prime, w);
                            y3Prime = returnY3(x1Prime, y1Prime, x2Prime, y2Prime, w);
                            x4Prime = returnX4(x1Prime, y1Prime, x2Prime, y2Prime, w);
                            y4Prime = returnY4(x1Prime, y1Prime, x2Prime, y2Prime, w);
                        } else {
                            x1Prime = linkPrime.getLastSegment().getStartX() * Parameters.pixelPerMeter;
                            y1Prime = linkPrime.getLastSegment().getStartY() * Parameters.pixelPerMeter;
                            x2Prime = linkPrime.getLastSegment().getEndX() * Parameters.pixelPerMeter;
                            y2Prime = linkPrime.getLastSegment().getEndY() * Parameters.pixelPerMeter;

                            //double w = 2 * pixelPerFootpathStrip + (linkPrime.getSegment(linkPrime.getNumberOfSegments() - 1).numberOfStrips() - 2) * pixelPerStrip;
                            double w = linkPrime.getLastSegment().getSegmentWidth() * Parameters.pixelPerMeter;


                            x3Prime = returnX3(x1Prime, y1Prime, x2Prime, y2Prime, w);
                            y3Prime = returnY3(x1Prime, y1Prime, x2Prime, y2Prime, w);
                            x4Prime = returnX4(x1Prime, y1Prime, x2Prime, y2Prime, w);
                            y4Prime = returnY4(x1Prime, y1Prime, x2Prime, y2Prime, w);

                            x1Prime = x2Prime;
                            y1Prime = y2Prime;
                            x3Prime = x4Prime;
                            y3Prime = y4Prime;
                        }

                        double[] one = new double[]{x1, y1, x1Prime, y1Prime};
                        double[] two = new double[]{x1, y1, x3Prime, y3Prime};
                        double[] three = new double[]{x3, y3, x1Prime, y1Prime};
                        double[] four = new double[]{x3, y3, x3Prime, y3Prime};
                        lineList.add(one);
                        lineList.add(two);
                        lineList.add(three);
                        lineList.add(four);
                    }
                }
            }
        }
        for (int i = 0; i < lineList.size(); i++) {
            boolean doIntersect = false;
            for (int j = 0; j < lineList.size(); j++) {
                if (i != j) {
                    if (doIntersect(lineList.get(i)[0], lineList.get(i)[1],
                            lineList.get(i)[2], lineList.get(i)[3],
                            lineList.get(j)[0], lineList.get(j)[1],
                            lineList.get(j)[2], lineList.get(j)[3])) {
                        doIntersect = true;
                        break;
                    }
                }
            }
            if (!doIntersect) {
                g2d.drawLine((int) round(lineList.get(i)[0]),
                        (int) round(lineList.get(i)[1]),
                        (int) round(lineList.get(i)[2]),
                        (int) round(lineList.get(i)[3]));
            }
        }
        for (Link link : linkList) {
            Segment segment = link.getFirstSegment();
            for (int j = 1; j < link.getNumberOfSegments(); j++) {
                double x1 = segment.getStartX() * Parameters.pixelPerMeter;
                double y1 = segment.getStartY() * Parameters.pixelPerMeter;
                double x2 = segment.getEndX() * Parameters.pixelPerMeter;
                double y2 = segment.getEndY() * Parameters.pixelPerMeter;

                //double w = 2 * pixelPerFootpathStrip + (segment.numberOfStrips() - 2) * pixelPerStrip;
                double w = segment.getSegmentWidth() * Parameters.pixelPerMeter;

                double x3 = returnX3(x1, y1, x2, y2, w);
                double y3 = returnY3(x1, y1, x2, y2, w);
                double x4 = returnX4(x1, y1, x2, y2, w);
                double y4 = returnY4(x1, y1, x2, y2, w);

                double x1Prime = link.getSegment(j).getStartX() * Parameters.pixelPerMeter;
                double y1Prime = link.getSegment(j).getStartY() * Parameters.pixelPerMeter;
                double x2Prime = link.getSegment(j).getEndX() * Parameters.pixelPerMeter;
                double y2Prime = link.getSegment(j).getEndY() * Parameters.pixelPerMeter;

                //double wPrime = 2 * pixelPerFootpathStrip + (link.getSegment(j).numberOfStrips() - 2) * pixelPerStrip;
                double wPrime = link.getSegment(j).getSegmentWidth() * Parameters.pixelPerMeter;

                double x3Prime = returnX3(x1Prime, y1Prime, x2Prime, y2Prime, wPrime);
                double y3Prime = returnY3(x1Prime, y1Prime, x2Prime, y2Prime, wPrime);
                double x4Prime = returnX4(x1Prime, y1Prime, x2Prime, y2Prime, wPrime);
                double y4Prime = returnY4(x1Prime, y1Prime, x2Prime, y2Prime, wPrime);

                g2d.drawLine((int) round(x2), (int) round(y2), (int) round(x1Prime), (int) round(y1Prime));
                g2d.drawLine((int) round(x4), (int) round(y4), (int) round(x3Prime), (int) round(y3Prime));

                segment = link.getSegment(j);
            }
        }
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        Parameters.showProgressSlider.setValue(Parameters.simulationStep);
        if (!Parameters.TRACE_MODE) {
            try {
                traceWriter.write("SimulationStep: " + Parameters.simulationStep);
                traceWriter.newLine();
            } catch (IOException ex) {
                ex.printStackTrace();
            }
        }

        processor.manualProcess(jFrame);
    }

    @Override
    public void mousePressed(MouseEvent e) {
        referenceX = e.getX();
        referenceY = e.getY();
    }

    @Override
    public void mouseDragged(MouseEvent e) {
        translateX += (e.getX() - referenceX) * 30;
        translateY += (e.getY() - referenceY) * 30;
        referenceX = e.getX();
        referenceY = e.getY();
        if (!Parameters.TRACE_MODE) {
            repaint();
        }
        if (Parameters.DEBUG_MODE) {
            System.out.println(translateX + " " + translateY);
        }
    }

    @Override
    public void mouseClicked(MouseEvent me) {
        // System.out.println("Mouse Clicked");
    }

    @Override
    public void mouseReleased(MouseEvent me) {
    }

    @Override
    public void mouseEntered(MouseEvent me) {
        // System.out.println("Mouse Entered");
    }

    @Override
    public void mouseExited(MouseEvent me) {
        // System.out.println("Mouse Exited");
    }

    @Override
    public void mouseMoved(MouseEvent me) {
    }

    LineNumberReader getTraceReader() {
        return traceReader;
    }

    void setTraceReader(LineNumberReader traceReader) {
        this.traceReader = traceReader;
    }

    @Override
    public void mouseWheelMoved(MouseWheelEvent e) {
        int notches = e.getWheelRotation();
        double newScaleValue = scale - notches * 0.004;
        scale = Math.min(1.0, Math.max(0.001, newScaleValue));
    }
}
