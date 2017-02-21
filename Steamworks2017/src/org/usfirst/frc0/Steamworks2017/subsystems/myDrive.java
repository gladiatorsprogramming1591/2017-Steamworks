package org.usfirst.frc0.Steamworks2017.subsystems;

import org.usfirst.frc0.Steamworks2017.Robot;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.communication.FRCNetworkCommunicationsLibrary.tInstances;
//import edu.wpi.first.wpilibj.communication.FRCNetworkCommunicationsLibrary.tResourceType;
//import edu.wpi.first.wpilibj.communication.UsageReporting;

/**
 * Utility class for handling Robot drive based on a definition of the motor
 * configuration. The robot drive class handles basic driving for a robot.
 * Currently, 2 and 4 motor tank and mecanum drive trains are supported. In the
 * future other drive types like swerve might be implemented. Motor channel
 * numbers are supplied on creation of the class. Those are used for either the
 * drive function (intended for hand created drive code, such as autonomous) or
 * with the Tank/Arcade functions intended to be used for Operator Control
 * driving.
 */
public class myDrive extends RobotDrive {
	
    static final int kFrontLeft_val = 0;
    static final int kFrontRight_val = 1;
    static final int kRearLeft_val = 2;
    static final int kRearRight_val = 3;
	private static final double SCALEFACTOR = 1.2;

	public myDrive(SpeedController frontLeftMotor, SpeedController rearLeftMotor, SpeedController frontRightMotor,
			SpeedController rearRightMotor) {
		super(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
	}
	public void mecanumDrive_raw(double fl, double fr, double bl, double br) {
	    m_frontLeftMotor.set(fl);
	    m_frontRightMotor.set(fr);
	    m_rearLeftMotor.set(bl);
	    m_rearRightMotor.set(br);
	}

	@Override
	public void mecanumDrive_Cartesian(double x, double y, double rotation, double gyroAngle) {
//	    if (!kMecanumCartesian_Reported) {
//	      UsageReporting.report(tResourceType.kResourceType_RobotDrive, getNumMotors(),
//	          tInstances.kRobotDrive_MecanumCartesian);
//	      kMecanumCartesian_Reported = true;
//	    }
	    double X1 = x;
	    double Y1 = y;
	    double X2 = rotation;
	    
	    // Negate y for the joystick.
	    Y1 = -Y1;
	    // Compenstate for gyro angle.
	    double rotated[] = rotateVector(X1, Y1, gyroAngle);
	    X1 = rotated[0];
	    Y1 = rotated[1];

	    double wheelSpeeds[] = new double[kMaxNumberOfMotors];
	    wheelSpeeds[kFrontLeft_val] = Y1 + X2 + X1;
	    wheelSpeeds[kFrontRight_val] = Y1 - X2 - X1;
	    wheelSpeeds[kRearLeft_val] = Y1 + X2 - X1;
	    wheelSpeeds[kRearRight_val] = Y1 - X2 + X1;
	    
	    SmartDashboard.putNumber("magnitude", Robot.oi.driveStick.getMagnitude());

	    normalize(wheelSpeeds);
	    m_frontLeftMotor.set(wheelSpeeds[kFrontLeft_val] * m_maxOutput);
	    m_frontRightMotor.set(wheelSpeeds[kFrontRight_val] * m_maxOutput * SCALEFACTOR);
	    m_rearLeftMotor.set(wheelSpeeds[kRearLeft_val] * m_maxOutput);
	    m_rearRightMotor.set(wheelSpeeds[kRearRight_val] * m_maxOutput * SCALEFACTOR);
	    
//	    if (m_syncGroup != 0) {
//	      CANTalon.FeedbackDeviceStatus;
//	    }

	    if (m_safetyHelper != null)
	      m_safetyHelper.feed();
	  }
	

//	@Override
//	public void mecanumDrive_Polar(double magnitude, double direction, double rotation) {
//	    if (!kMecanumPolar_Reported) {
//	      UsageReporting.report(tResourceType.kResourceType_RobotDrive, getNumMotors(),
//	          tInstances.kRobotDrive_MecanumPolar);
//	      kMecanumPolar_Reported = true;
//	    }
//	    double frontLeftSpeed, rearLeftSpeed, frontRightSpeed, rearRightSpeed;
//	    // Normalized for full power along the Cartesian axes.
//	    magnitude = limit(magnitude) * Math.sqrt(2.0);
//	    // The rollers are at 45 degree angles.
//	    double dirInRad = (direction + 45.0) * 3.14159 / 180.0;
//	    double cosD = Math.cos(dirInRad);
//	    double sinD = Math.sin(dirInRad);
//
//	    double wheelSpeeds[] = new double[kMaxNumberOfMotors];
//	    wheelSpeeds[kFrontLeft_val] = (sinD * magnitude + rotation);
//	    wheelSpeeds[kFrontRight_val] = (cosD * magnitude - rotation);
//	    wheelSpeeds[kRearLeft_val] = (cosD * magnitude + rotation);
//	    wheelSpeeds[kRearRight_val] = (sinD * magnitude - rotation);
//
//	    normalize(wheelSpeeds);
//
//	    m_frontLeftMotor.set(wheelSpeeds[kFrontLeft_val] * m_maxOutput, m_syncGroup);
//	    m_frontRightMotor.set(wheelSpeeds[kFrontRight_val] * m_maxOutput, m_syncGroup);
//	    m_rearLeftMotor.set(wheelSpeeds[kRearLeft_val] * m_maxOutput, m_syncGroup);
//	    m_rearRightMotor.set(wheelSpeeds[kRearRight_val] * m_maxOutput, m_syncGroup);
//
//	    if (this.m_syncGroup != 0) {
//	      CANJaguar.updateSyncGroup(m_syncGroup);
//	    }
//
//	    if (m_safetyHelper != null)
//	      m_safetyHelper.feed();
//	  }
}
