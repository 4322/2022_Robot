package frc.robot.FiringSolution;

public class FiringSolution {
    private double flywheelSpeed;
    private double hoodPosition;
    private double distance;


    public FiringSolution(double flywheelSpeed, double hoodPosition, double distanceIn) {
        this.flywheelSpeed = flywheelSpeed;
        this.hoodPosition = hoodPosition;
        this.distance = distanceIn;
    }

    public FiringSolution(double flywheelSpeed, double hoodPosition) {
        this.flywheelSpeed = flywheelSpeed;
        this.hoodPosition = hoodPosition;
    }

    public double getflywheelSpeed() { return flywheelSpeed; }
    public double gethoodPosition() { return hoodPosition; }
    public double getDistance() { return distance; }
}