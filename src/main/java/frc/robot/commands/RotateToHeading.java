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

public class RotateToHeading extends Command {
  PID rotatePID = new PID(.2, 0, 0); //change speed: .35 for 90, .6 for 30, inter/extrapolate from there
	int heading;
	double speed;
	long stopTime;
	
    public RotateToHeading(int heading, double speed) {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.driveTrain);
        this.heading = heading;
        this.speed = speed;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	rotatePID.setTarget(heading);
    	stopTime = System.currentTimeMillis() + 3000;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double correction = rotatePID.getCorrection(Robot.sensors.getYaw());
    	if (correction > 1) {
    		correction = 1;
    	}
    	
    	if (correction < -1) {
    		correction = -1;
    	}
    	
    	Robot.driveTrain.setPower(-speed * correction, speed * correction);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	Robot.sensors.updatePosition();
    	boolean headingReached = Math.abs(Robot.sensors.getYaw() - heading) <= 2;
    	boolean isDone = headingReached && Robot.sensors.isMoving == false;
    	return headingReached || System.currentTimeMillis() > stopTime;   
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveTrain.setPower(0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	
    }
}
