package frc.robot.FiringSolution;

public class FiringSolution {
    private double flywheelSpeed;
    private double hoodPosition;
    private double kickerSpeed;
    private double aimingToleranceDeg;
    private double distance;

    public FiringSolution(double kickerSpeed, double flywheelSpeed, double hoodPosition, double aimingToleranceDeg, double distanceIn) {
      this.kickerSpeed = kickerSpeed;
      this.flywheelSpeed = flywheelSpeed;
      this.hoodPosition = hoodPosition;
      this.aimingToleranceDeg = aimingToleranceDeg;
      this.distance = distanceIn;
    }

    public double getFlywheelSpeed() { return flywheelSpeed; }
    public double getKickerSpeed() { return kickerSpeed; }
    public double getHoodPosition() { return hoodPosition; }
    public double getAimingToleranceDeg() { return aimingToleranceDeg; }
    public double getDistance() { return distance; }
}