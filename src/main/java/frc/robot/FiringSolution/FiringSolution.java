package frc.robot.FiringSolution;

import frc.robot.utility.OrangeMath;

public class FiringSolution {
  private double flywheelSpeed;
  private double hoodPosition;
  private double kickerSpeed;
  private double aimingToleranceDeg;
  private double distance;

  public FiringSolution(double kickerSpeed, double flywheelSpeed, double hoodPosition,
      double aimingToleranceDeg, double distanceIn) {
    this.kickerSpeed = kickerSpeed;
    this.flywheelSpeed = flywheelSpeed;
    this.hoodPosition = hoodPosition;
    this.aimingToleranceDeg = aimingToleranceDeg;
    this.distance = distanceIn;
  }

  // See: https://www.geeksforgeeks.org/overriding-equals-method-in-java/
  @Override
  public boolean equals(Object o) {

    // If the object is compared with itself then return true
    if (o == this) {
      return true;
    }

    /*
     * Check if o is an instance of FiringSolution or not "null instanceof [type]" also returns
     * false
     */
    if (!(o instanceof FiringSolution)) {
      return false;
    }

    // typecast o to FiringSolution so that we can compare data members
    FiringSolution c = (FiringSolution) o;

    return OrangeMath.equalToTwoDecimal(c.getAimingToleranceDeg(), aimingToleranceDeg)
        && OrangeMath.equalToTwoDecimal(c.getDistance(), distance)
        && OrangeMath.equalToTwoDecimal(c.getFlywheelSpeed(), flywheelSpeed)
        && OrangeMath.equalToTwoDecimal(c.getHoodPosition(), hoodPosition)
        && OrangeMath.equalToTwoDecimal(c.getKickerSpeed(), kickerSpeed);
  }

  public double getFlywheelSpeed() {
    return flywheelSpeed;
  }

  public double getKickerSpeed() {
    return kickerSpeed;
  }

  public double getHoodPosition() {
    return hoodPosition;
  }

  public double getAimingToleranceDeg() {
    return aimingToleranceDeg;
  }

  public double getDistance() {
    return distance;
  }
}
