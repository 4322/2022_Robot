package frc.robot.subsystems.SwerveDrive;

import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.ControlMode;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class TalonFXModule extends ControlModule {
	
	private WPI_TalonFX m_rotation; 
	private WPI_TalonFX m_wheel;
	
	private double wheelDiameter = 4.0;
	private double rotationDiameter = 4.0;
	private int ticksPerRev = 4096;
	
	public TalonFXModule(WPI_TalonFX rotation, WPI_TalonFX wheel, WheelPosition pos) {
		super(pos);
		m_rotation = rotation;
		m_wheel = wheel;

/*Need to configure both the drive and rotation motor controllers using the following example:
    private void configDriveMaster(WPI_TalonFX talon) {
        talon.configFactoryDefault();

        talon.configClosedloopRamp(Config.driveClosedLoopRampRate, Config.motorControllerConfigTimeoutMs);
        talon.configOpenloopRamp(Config.driveOpenLoopRampRate, Config.motorControllerConfigTimeoutMs);
        talon.config_kP(0, Config.driveVelocityP, Config.motorControllerConfigTimeoutMs);
        talon.config_kI(0, Config.driveVelocityI, Config.motorControllerConfigTimeoutMs);
        talon.config_IntegralZone(0, Config.driveVelocityIntegralZone, Config.motorControllerConfigTimeoutMs);
        talon.config_kD(0, Config.driveVelocityD, Config.motorControllerConfigTimeoutMs);
        talon.config_kF(0, Config.driveVelocityF, Config.motorControllerConfigTimeoutMs);

        talon.setNeutralMode(NeutralMode.Brake);
        talon.setInverted(true);
        talon.setSensorPhase(false);	
    }

	rotation motor PID paraemters:
	private final double kP = (0.007);
    private final double kD = (0.00024);

	drive motor PID parameters:
	private final double kP = (0.06);
	private final double kD = (0.0);
    private final double kD = (0.0);
    private final double kF = (0.083);

	Both motors:
	configOpenloopRamp(0.08);
	configClosedloopRamp(0.08);
    configVoltageCompSaturation(12);
    enableVoltageCompensation(true);

	For rotation motor only to use external encoder:
	configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

  * Configure the current limits that will be used
  * Stator Current is the current that passes through the motor stators.
  *  Use stator current limits to limit rotor acceleration/heat production
  * Supply Current is the current that passes into the controller from the supply
  *  Use supply current limits to prevent breakers from tripping
  *
  * Drive motor:
  *                                                               enabled | Limit(amp) | Trigger Threshold(amp) | Trigger Threshold Time(s)
  configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,      40,                45,                1.0));
  configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,      40,                45,                0.5));

  * Rotation motor:
  *                                                               enabled | Limit(amp) | Trigger Threshold(amp) | Trigger Threshold Time(s)
  configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,      40,                45,                1.0));
  configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,      30,                35,                0.5));
  */
}
	
	public void setSpeedAndAngle(Joystick drive, Joystick rotate){
		m_wheel.set(ControlMode.PercentOutput, SwerveHelper.getSpeedValue(drive, rotate, position.wheelNumber));
		m_rotation.set(ControlMode.Position, SwerveHelper.getAngleValue(drive, rotate, position.wheelNumber));
	}
	
	public void setRotationPID(double kp, double ki, double kd){
		m_rotation.config_kP(0, kp, 0);
		m_rotation.config_kI(1, ki, 0);
		m_rotation.config_kD(2, kd, 0);
	}

	public double getRotationP(){
		return m_rotation.configGetParameter(0, 0, 0);
	}

	public double getRotationI(){
		return m_rotation.configGetParameter(1, 0, 0);
	}

	public double getRotationD(){
		return m_rotation.configGetParameter(2, 0, 0);
	}
	
	@Override
	public double getAngle(){
		return (m_rotation.getSelectedSensorPosition(0) *  Math.PI * wheelDiameter);
	}
	
	@Override
	public double getDistance(){
		return (m_wheel.getSelectedSensorPosition(0) *  Math.PI * wheelDiameter) / ticksPerRev;
	}

	@Override
	public double getVelocity() {
		return m_wheel.getSelectedSensorVelocity(0) * 600/ticksPerRev;
	}

	@Override
	public double getAcceleration() {
		return super.getAcceleration();
	}

	public double getWheelDiameter() {
		return wheelDiameter;
	}

	public void setWheelDiameter(double wheelDiameter) {
		this.wheelDiameter = wheelDiameter;
	}

	public double getRotationDiameter() {
		return rotationDiameter;
	}

	public void setRotationDiameter(double rotationDiameter) {
		this.rotationDiameter = rotationDiameter;
	}

	public int getTicksPerRev() {
		return ticksPerRev;
	}

	public void setTicksPerRev(int ticksPerRev) {
		this.ticksPerRev = ticksPerRev;
	}	

}
