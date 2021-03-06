/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ClimberVertical extends Command {
  double power;
  
  public ClimberVertical(double power) {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.climber);
    this.power = power;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.climber.setVerticalPower(power);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    double encoderPos = Robot.climber.verticalMotor.getSelectedSensorPosition();
    if ((power < 0) && (encoderPos < -7229539)) {
      return true;
    }
    if ((power > 0) && (encoderPos > 5000)) {
      return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.climber.setVerticalPower(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.climber.setVerticalPower(0);
  }
}
