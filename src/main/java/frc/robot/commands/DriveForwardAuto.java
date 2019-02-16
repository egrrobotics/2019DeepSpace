/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.utils.PID;

public class DriveForwardAuto extends Command {
  PID drivePID = new PID(.02, 0, 0);
	PID stopPID; // .0006 for 20% speed, .000125 for 30% speed
	double maxSpeed;
	int distance;
	double heading;
	double speed = 0;
	double encoderTarget;
	double stopTime;


	public DriveForwardAuto(double speed, int distance, double heading) {
		requires(Robot.driveTrain);
		this.maxSpeed = speed;
		this.distance = distance;
		this.heading = heading;
		this.stopPID = new PID(.04, 0, 0);
	}

	public DriveForwardAuto(double speed, int distance, double heading, double stopP) {
		requires(Robot.driveTrain);
		this.maxSpeed = speed;
		this.distance = distance;
		this.heading = heading;
		this.stopPID = new PID(stopP, 0, 0);
  }
  
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    encoderTarget = Robot.sensors.getRightEncoder() + distance * Robot.sensors.encoderCountsPerInch;
		drivePID.setTarget(heading);
		stopPID.setTarget(encoderTarget);
		stopTime = System.currentTimeMillis() +7000;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double stopCorrection = stopPID.getCorrection(Robot.sensors.getRightEncoder());
		if (stopCorrection > 1) {
			stopCorrection = 1;
		}
		double correction = drivePID.getCorrection(Robot.sensors.getYaw());
		if (speed < maxSpeed) {
			speed += 0.04;
		}
		if (encoderTarget - Robot.sensors.getRightEncoder() < 300) {
			correction = 0;
		}
		
		Robot.driveTrain.setPower((speed - correction) * stopCorrection, (speed + correction) * stopCorrection);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.sensors.getRightEncoder() > encoderTarget - 30 || System.currentTimeMillis() > stopTime;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveTrain.setPower(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
