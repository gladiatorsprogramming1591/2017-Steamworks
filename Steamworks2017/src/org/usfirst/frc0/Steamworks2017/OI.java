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

import org.usfirst.frc0.Steamworks2017.commands.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

import org.usfirst.frc0.Steamworks2017.subsystems.*;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	//// CREATING BUTTONS
	// One type of button is a joystick button which is any button on a joystick.
	// You create one by telling it which joystick it's on and which button
	// number it is.
	// Joystick stick = new Joystick(port);
	// Button button = new JoystickButton(stick, buttonNumber);

	// There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS
	// Once you have a button, it's trivial to bind it to a button in one of
	// three ways:

	// Start the command when the button is pressed and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenPressed(new ExampleCommand());

	// Run the command while the button is being held down and interrupt it once
	// the button is released.
	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released  and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());


	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public JoystickButton flapCloseButton;
    public JoystickButton flapOpenButton;
    public JoystickButton extractPlungerButton;
    public JoystickButton insertPlungerButton;
    public Joystick driveStick;
    public Joystick rotateStick;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

	public OI() {
		// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

        rotateStick = new Joystick(1);
        
        driveStick = new Joystick(0);
        
        insertPlungerButton = new JoystickButton(driveStick, 1);
        insertPlungerButton.whenPressed(new InsertPlunger());
        extractPlungerButton = new JoystickButton(driveStick, 2);
        extractPlungerButton.whenPressed(new ExtractPlunger());
        flapOpenButton = new JoystickButton(driveStick, 2);
        flapOpenButton.whenPressed(new FlapOpen());
        flapCloseButton = new JoystickButton(driveStick, 1);
        flapCloseButton.whenPressed(new FlapClose());


        // SmartDashboard Buttons
        SmartDashboard.putData("BasicDrive: 5 sec 1/2 speed", new BasicDrive(true, 5, .5, 0, 0, 0));
        SmartDashboard.putData("DriveFowardTest", new DriveFowardTest());
        SmartDashboard.putData("DriveBackwardTest", new DriveBackwardTest());
        SmartDashboard.putData("WinchUp: 5 sec 1/2 speed", new WinchUp(5, 0.5));
        SmartDashboard.putData("ManualDrive", new ManualDrive());
        SmartDashboard.putData("InsertPlunger", new InsertPlunger());
        SmartDashboard.putData("ExtractPlunger", new ExtractPlunger());
        SmartDashboard.putData("FlapClose", new FlapClose());
        SmartDashboard.putData("FlapOpen", new FlapOpen());
        SmartDashboard.putData("ReadPixyPackets", new ReadPixyPackets());
        SmartDashboard.putData("ResetPixy", new ResetPixy());
        SmartDashboard.putData("RunWinchPercentVBus: Half", new RunWinchPercentVBus(0.5));
        SmartDashboard.putData("RunWinchPercentVBus: ThreeQuarters", new RunWinchPercentVBus(0.75));
        SmartDashboard.putData("RunWinchPercentVBus: NegHalf", new RunWinchPercentVBus(-0.5));
        SmartDashboard.putData("RunWinchPercentVBus: NegThreeQuarters", new RunWinchPercentVBus(-0.75));
        SmartDashboard.putData("EncoderBRPercentVBus: ThreeQuartersBR", new EncoderBRPercentVBus(0.75));
        SmartDashboard.putData("EncoderBRPercentVBus: NegThreeQuartersBR", new EncoderBRPercentVBus(-0.75));
        SmartDashboard.putData("EncoderBLPercentVBus: ThreeQuartersBL", new EncoderBLPercentVBus(0.75));
        SmartDashboard.putData("EncoderBLPercentVBus: NegThreeQuartersBL", new EncoderBLPercentVBus(-0.75));
        SmartDashboard.putData("EncoderFRPercentVBus", new EncoderFRPercentVBus());
        SmartDashboard.putData("EncoderFLPercentVBus: ThreeQuartersFL", new EncoderFLPercentVBus(0.75));
        SmartDashboard.putData("EncoderFLPercentVBus: NegThreeQuartersFL", new EncoderFLPercentVBus(-0.75));
        SmartDashboard.putData("DisplayGearPosition", new DisplayGearPosition());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
	}

	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
    public Joystick getDriveStick() {
        return driveStick;
    }

    public Joystick getRotateStick() {
        return rotateStick;
    }


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
}

