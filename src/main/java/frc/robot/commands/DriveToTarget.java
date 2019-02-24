/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class DriveToTarget extends Command {

  public DriveToTarget() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // currentYaw = Robot.sensors.getYaw();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (Robot.tapeTable.getEntry("tapeInView").getBoolean(false)) {
      double pairCenterX = Robot.tapeTable.getEntry("pairCenterX").getDouble(0);
      double distToTape = Robot.tapeTable.getEntry("estDist").getDouble(30);
      System.out.println(pairCenterX);

      double baseSpeed = 0.00444 * distToTape + 0.26667; // Fast (far) -> slow (close)
      // Set turnSpeed as % of baseSpeed:
      double turnSpeed = baseSpeed * (-0.00222 * distToTape + 0.56667); // Smooth (far) -> agressive (close)
      Robot.driveTrain.setPower(
        baseSpeed + turnSpeed * (2 * pairCenterX),
        baseSpeed - turnSpeed * (2 * pairCenterX)
      );
    } else {
      System.out.println("NO TAPE IN VIEW");
      Robot.driveTrain.setPower(0.4);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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
    Robot.driveTrain.setPower(0);
  }
}
