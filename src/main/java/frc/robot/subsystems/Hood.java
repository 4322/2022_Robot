package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hood extends SubsystemBase{
    
    private WPI_TalonFX hood;

    //SHUFFLEBOARD
    private ShuffleboardTab tab = Shuffleboard.getTab("Hood");

    private NetworkTableEntry isHomeIndicator =
    tab.add("Is @ home", false)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .withPosition(0,2)
    .withSize(1,1)
    .getEntry();

    public Hood() {
        hood = new WPI_TalonFX(Constants.HoodConstants.hoodTalon_ID);

        hood.configFactoryDefault();
        hood.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
                                                  Constants.HoodConstants.kPIDLoopIdx,
                                                  Constants.HoodConstants.kTimeoutMs);
        hood.setSensorPhase(Constants.HoodConstants.kSensorPhase);
    
        /* Config the peak and nominal outputs */
            hood.configNominalOutputForward(Constants.HoodConstants.minForwardPower, Constants.HoodConstants.kTimeoutMs);
            hood.configNominalOutputReverse(Constants.HoodConstants.minReversePower, Constants.HoodConstants.kTimeoutMs);
            hood.configPeakOutputForward(Constants.HoodConstants.maxForwardPower, Constants.HoodConstants.kTimeoutMs);
            hood.configPeakOutputReverse(Constants.HoodConstants.maxReversePower, Constants.HoodConstants.kTimeoutMs);        
    }
}
