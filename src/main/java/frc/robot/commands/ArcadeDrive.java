/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ArcadeDrive extends Command {
  double throttle;
  double wheel; 
  double leftPower;
  double rightPower;
  public ArcadeDrive() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

  public double deadBand(double x){
    if (Math.abs(x)<.2){
      return 0;
    }else{
      return x;
    }
  }
  
  
  public double clip(double x){
    
    if (x>.75) return .75;
    if (x<-.75) return -.75;
    return x;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    throttle = deadBand(-Robot.oi.driver.getRawAxis(1));
    wheel = deadBand(Robot.oi.driver.getRawAxis(4));
    leftPower = clip(throttle + wheel);
    rightPower = clip(throttle - wheel);
    Robot.driveTrain.setPower(leftPower, rightPower);
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
