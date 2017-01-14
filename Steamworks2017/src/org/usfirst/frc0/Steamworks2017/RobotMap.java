// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc0.Steamworks2017;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.TalonSRX;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public static SpeedController driveTrainSpeedControllerBR;
    public static SpeedController driveTrainSpeedControllerBL;
    public static SpeedController driveTrainSpeedControllerFR;
    public static SpeedController driveTrainSpeedControllerFL;
    public static RobotDrive driveTrainRobotDrive;
    public static Encoder driveTrainEncoderFL;
    public static Encoder driveTrainEncoderFR;
    public static Encoder driveTrainEncoderBL;
    public static Encoder driveTrainEncoderBR;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public static void init() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        driveTrainSpeedControllerBR = new TalonSRX(7);
        LiveWindow.addActuator("DriveTrain", "SpeedControllerBR", (TalonSRX) driveTrainSpeedControllerBR);
        
        driveTrainSpeedControllerBL = new TalonSRX(5);
        LiveWindow.addActuator("DriveTrain", "SpeedControllerBL", (TalonSRX) driveTrainSpeedControllerBL);
        
        driveTrainSpeedControllerFR = new TalonSRX(6);
        LiveWindow.addActuator("DriveTrain", "SpeedControllerFR", (TalonSRX) driveTrainSpeedControllerFR);
        
        driveTrainSpeedControllerFL = new TalonSRX(3);
        LiveWindow.addActuator("DriveTrain", "SpeedControllerFL", (TalonSRX) driveTrainSpeedControllerFL);
        
        driveTrainRobotDrive = new RobotDrive(driveTrainSpeedControllerFL, driveTrainSpeedControllerBL,
              driveTrainSpeedControllerFR, driveTrainSpeedControllerBR);
        
        driveTrainRobotDrive.setSafetyEnabled(true);
        driveTrainRobotDrive.setExpiration(0.1);
        driveTrainRobotDrive.setSensitivity(0.5);
        driveTrainRobotDrive.setMaxOutput(1.0);

        driveTrainEncoderFL = new Encoder(0, 1, false, EncodingType.k4X);
        LiveWindow.addSensor("DriveTrain", "EncoderFL", driveTrainEncoderFL);
        driveTrainEncoderFL.setDistancePerPulse(1.0);
        driveTrainEncoderFL.setPIDSourceType(PIDSourceType.kRate);
        driveTrainEncoderFR = new Encoder(2, 3, false, EncodingType.k4X);
        LiveWindow.addSensor("DriveTrain", "EncoderFR", driveTrainEncoderFR);
        driveTrainEncoderFR.setDistancePerPulse(1.0);
        driveTrainEncoderFR.setPIDSourceType(PIDSourceType.kRate);
        driveTrainEncoderBL = new Encoder(4, 5, false, EncodingType.k4X);
        LiveWindow.addSensor("DriveTrain", "EncoderBL", driveTrainEncoderBL);
        driveTrainEncoderBL.setDistancePerPulse(1.0);
        driveTrainEncoderBL.setPIDSourceType(PIDSourceType.kRate);
        driveTrainEncoderBR = new Encoder(6, 7, false, EncodingType.k4X);
        LiveWindow.addSensor("DriveTrain", "EncoderBR", driveTrainEncoderBR);
        driveTrainEncoderBR.setDistancePerPulse(1.0);
        driveTrainEncoderBR.setPIDSourceType(PIDSourceType.kRate);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }
}
