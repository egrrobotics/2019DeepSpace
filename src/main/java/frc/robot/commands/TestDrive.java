/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class TestDrive extends Command {
  double positiveValue; double negativeValue; double finalValue; // Control values.

  public TestDrive() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.testTrain);
  }

  private double DeadZone(double value, double zone){
    // Deadzones a raw controller value.
    if (Math.abs(value)<zone){
      value = 0;
    }
    return value;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Get & compute final power value.
    negativeValue = Robot.oi.driver.getRawAxis(2); positiveValue = Robot.oi.driver.getRawAxis(3);
    finalValue = positiveValue - negativeValue; finalValue = DeadZone(finalValue, .15);

    // Set motor power with that final value.
    Robot.testTrain.setPower(finalValue);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
