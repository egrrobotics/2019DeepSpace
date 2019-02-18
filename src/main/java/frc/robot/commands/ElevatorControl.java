/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ElevatorControl extends Command {

  public ElevatorControl() {
    requires(Robot.elevator);

    Robot.elevator.elevatorMotor.setSelectedSensorPosition(0);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double manualAdjust = Robot.oi.operator.getRawAxis(1);
    if (Math.abs(manualAdjust) > 0.1) {
      Robot.elevator.setTargetHeight(Robot.elevator.targetHeight + manualAdjust * -25);
    }

    // Prevent tipping
    if(Math.abs(Robot.sensors.getPitch()) > 10) {
      Robot.elevator.setTargetHeight(500);
    }

    Robot.elevator.moveTowardTarget();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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
