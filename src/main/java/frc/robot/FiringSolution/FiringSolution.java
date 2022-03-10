package frc.robot.FiringSolution;

public class FiringSolution {
    private double flywheelSpeed;
    private double hoodPosition;
    private double distance;


    public FiringSolution(double flywheelSpeed, double hoodPosition, double distanceFt) {
        this.flywheelSpeed = flywheelSpeed;
        this.hoodPosition = hoodPosition;
        this.distance = distanceFt;
    }

    public double getflywheelSpeed() { return flywheelSpeed; }
    public double gethoodPosition() { return hoodPosition; }
    public double getDistance() { return distance; }
}