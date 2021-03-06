/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Sensors extends Subsystem {
  private boolean usingTalonEncoders = true;	   		// <-- move constant to RobotMap (robot setup class)
	private Encoder leftEncoder = null;	// <-- move constant to RobotMap (robot setup class)
	private Encoder rightEncoder = null;	// <-- move constant to RobotMap (robot setup class)
	public Encoder elevatorEncoder = new Encoder(RobotMap.elevatorEncoderA, RobotMap.elevatorEncoderB, false, Encoder.EncodingType.k4X);
	//static Potentiometer pot = new AnalogPotentiometer(0, 360, 10); //Channel number for Analog input, scale factor 360 being the great, offset to add after scaling to prevent breakage (10 to 30 range) 
	static AHRS navX;
	int rightEncoderOffset = 0;
	int leftEncoderOffset = 0;
	float yawOffset = 0;
	double XCoordinate = 0;
	double YCoordinate = 0;
	double lastLeftEncoder = 0;
	double lastRightEncoder = 0;
	boolean encoderInvert = false;
	public double encoderCountsPerInch = RobotMap.encoderCountsPerInch;
	public boolean isMoving = false;
	public boolean leftEncoderMoving = false;
	public boolean rightEncoderMoving = false;
	
	
	public Sensors() {
		try {
			navX = new AHRS(Port.kMXP);
		}
		catch(RuntimeException ex) {
			DriverStation.reportError("error instantiating navX: " + ex.getMessage(), true);
		}
		this.resetSensors();
	}
	
	public float getYaw() {
		return getYaw(true);
	}
	public float getYaw(boolean preserve180) {
		float yaw;
		yaw = -(navX.getYaw() - yawOffset);
		if (preserve180){
			while(yaw > 180.0f) {
				yaw-=360.0f;
			}
			while (yaw < -180.0f) {
				yaw+=360.0f;
			}
		}
		return yaw;
	}
	
	public float getRoll() {
		float roll;
		roll = navX.getRoll();
		return roll;
	}
	
	public float getPitch() {
		float pitch;
		pitch = navX.getPitch();
		return pitch;
	}
	
	public int getLeftEncoderPosition() {
		if (this.usingTalonEncoders)
			return -Robot.driveTrain.left1.getSelectedSensorPosition(0);
		else
			return this.leftEncoder.get();
	}
	
	public int getRightEncoderPosition() {
		if (this.usingTalonEncoders)
			return -Robot.driveTrain.right1.getSelectedSensorPosition(0);
		else
			return this.rightEncoder.get();
	}
	
	public int getLeftEncoder() {
		int leftEncoderVal = this.getLeftEncoderPosition(); 	// this.leftTalon.getSelectedSensorPosition(0); // leftEncoder.get();
		int encoderValue = leftEncoderVal - leftEncoderOffset;
		if (this.usingTalonEncoders)
			return encoderValue;
		else
			return encoderValue*-1;		
	}
	
	public int getRightEncoder() {
		int rightEncoderVal = this.getRightEncoderPosition(); 	// this.rightTalon.getSelectedSensorPosition(0);  //rightEncoder.get();
		int encoderValue = rightEncoderVal - rightEncoderOffset;
		return encoderValue;
	}
	
	public double getElevatorEncoder() {
		return elevatorEncoder.get();
	}

	public void switchDirection() {
		Encoder temp = rightEncoder;
		rightEncoder = leftEncoder;
		leftEncoder = temp;
		encoderInvert = !encoderInvert;
	}
	
	public void resetSensors() {
		leftEncoderOffset = this.getLeftEncoderPosition();
		rightEncoderOffset = this.getRightEncoderPosition();
		
		lastLeftEncoder = 0;
		lastRightEncoder = 0;
		
		yawOffset = navX.getYaw();
	}
	
	public void resetYaw() {
		navX.reset();
	}
	
	public void resetLeftEncoder() {
		leftEncoder.reset();
	}
	
	public void resetRightEncoder() {
		rightEncoder.reset();
	}
	
	public void updatePosition() {
		double currentLeftEncoder = getLeftEncoder();
		double currentRightEncoder = getRightEncoder();
		double changeInLeftEncoder = currentLeftEncoder - lastLeftEncoder;
		double changeInRightEncoder = currentRightEncoder - lastRightEncoder;
		double encoderDistance = (changeInLeftEncoder + changeInRightEncoder)/2;
		double heading = Math.toRadians(getYaw());
		double changeInX = encoderDistance * Math.cos(heading);
		double changeInY = encoderDistance * Math.sin(heading);
		XCoordinate += changeInX;
		YCoordinate += changeInY;
		lastLeftEncoder = currentLeftEncoder;
		lastRightEncoder = currentRightEncoder;
		leftEncoderMoving = Math.abs(lastLeftEncoder-this.getLeftEncoder()) < 3;
		rightEncoderMoving = Math.abs(lastRightEncoder-this.getRightEncoder()) < 3;
		isMoving = leftEncoderMoving || rightEncoderMoving;
	}
	
	public double getXCoordinate() {
		return XCoordinate / encoderCountsPerInch;
	}
	
	public double getYCoordinate() {
		return YCoordinate / encoderCountsPerInch;
	}
	
	public void resetPosition() {
		XCoordinate = 0;
		YCoordinate = 0;
	}
	
	// ---------------------------------------------

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
}
