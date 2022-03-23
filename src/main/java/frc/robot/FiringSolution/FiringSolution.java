package frc.robot.FiringSolution;

public class FiringSolution {
    private double flywheelSpeed;
    private double hoodPosition;
    private double kickerSpeed;
    private double distance;

    public FiringSolution(double kickerSpeed, double flywheelSpeed, double hoodPosition, double distanceIn) {
      this.kickerSpeed = kickerSpeed;
      this.flywheelSpeed = flywheelSpeed;
      this.hoodPosition = hoodPosition;
      this.distance = distanceIn;
    }

    public FiringSolution(double kickerSpeed, double flywheelSpeed, double hoodPosition) {
      this.kickerSpeed = kickerSpeed;
      this.flywheelSpeed = flywheelSpeed;
      this.hoodPosition = hoodPosition;
    }

    public double getFlywheelSpeed() { return flywheelSpeed; }
    public double getKickerSpeed() { return kickerSpeed; }
    public double getHoodPosition() { return hoodPosition; }
    public double getDistance() { return distance; }
}