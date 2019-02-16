/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ElevatorSetHeight extends Command {
  double targetHeight;

  public ElevatorSetHeight(double height) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    requires(Robot.elevator);
    targetHeight = height;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double error = targetHeight - Robot.elevator.getCurrentHeight();
    double power = error * 0.005;
    power = Math.min(0.5, power);
    power = Math.max(-0.5, power);
    Robot.elevator.setPower(power);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Math.abs(Robot.elevator.getCurrentHeight() - targetHeight) < 5;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.elevator.setPower(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.elevator.setPower(0);
  }
}
