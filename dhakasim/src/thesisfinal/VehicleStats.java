package thesisfinal;

public class VehicleStats {
    private int vehicleId;
    private double[] accelerations;
    private double[]  speeds;
    private Point2D[] trajectory;

    VehicleStats(int vehicleId, int simulationEndTime) {
        this.vehicleId = vehicleId;
        accelerations = new double[simulationEndTime];
        speeds = new double[simulationEndTime];
        trajectory = new Point2D[simulationEndTime];
    }

    void addAcceleration(double acceleration, int simulationStep) {
        accelerations[simulationStep] = acceleration;
    }

    void addSpeed(double speed, int simulationStep) {
        speeds[simulationStep] = speed;
    }

    void addPointOnTrajectory(Point2D point, int simulationStep) {
        trajectory[simulationStep] = point;
    }

    public int getVehicleId() {
        return vehicleId;
    }

    public double[] getSpeeds() {
        return speeds;
    }

    public Point2D[] getTrajectory() {
        return trajectory;
    }

    public double[] getXTrajectory() {
        double[] x_coords = new double[trajectory.length];
        for (int i = 0; i < trajectory.length; i++) {
            if(trajectory[i] == null) {
                x_coords[i] = 0;
            } else{
                x_coords[i] = trajectory[i].x;
            }

        }
        return x_coords;
    }

    public double[] getYTrajectory() {
        double[] y_coords = new double[trajectory.length];
        for (int i = 0; i < trajectory.length; i++) {
            if(trajectory[i] == null) {
                y_coords[i] = 0;
            } else{
                y_coords[i] = trajectory[i].y;
            }

        }
        return y_coords;
    }
}
