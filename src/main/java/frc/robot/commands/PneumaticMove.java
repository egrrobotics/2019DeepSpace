/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class PneumaticMove extends Command {
  public enum PneumaticDirection {
    FORWARD, BACKWARD
  }

  private PneumaticDirection direction;

  public PneumaticMove(PneumaticDirection direction) {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.dummyPneumatic);
    
    this.direction = direction;
  }

  public void start() {
    switch(direction) {
      case FORWARD:
        Robot.dummyPneumatic.extend();
        break;
      case BACKWARD:
        Robot.dummyPneumatic.reverse();
        break;
    }
  }

  public void stop() {
    Robot.dummyPneumatic.stop();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    stop();
  }
}