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
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

public class DriveToTarget extends Command {

  // NetworkTable tapeTable;

  double maxVel = 2.8;
  
  Waypoint[] points;
  Trajectory.Config config;
  Trajectory trajectory;
  TankModifier modifier;

  EncoderFollower left;
  EncoderFollower right;

  double leftPower;
  double rightPower;

  int execCount = 0;

  long lastExecTime = 0;

  public DriveToTarget() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    /*
    tapeTable = Robot.networkTableInstance.getTable("ReflectiveTapeContours");
    deltaYaw = (float) tapeTable.getEntry("estAngle").getDouble(0);
    startYaw = Robot.sensors.getYaw();
    currentYaw = Robot.sensors.getYaw();
    */

    long startTime = System.currentTimeMillis();

    System.out.println("Starting tank path generation.");

    double angle = Pathfinder.d2r(Robot.sensors.getYaw());
    points = new Waypoint[] {
      new Waypoint(0.0, 0.0, angle),
      new Waypoint(
        Math.cos(angle) * 2 + Math.cos(angle + Math.PI / 2) * 0,
        Math.sin(angle) * 2 + Math.sin(angle + Math.PI / 2) * 0,
        angle
      )
    };

    config = new Trajectory.Config(
      Trajectory.FitMethod.HERMITE_CUBIC,
      Trajectory.Config.SAMPLES_LOW,
      0.02, maxVel, 3.05, 50.0
    );

    trajectory = Pathfinder.generate(points, config);
    modifier = new TankModifier(trajectory).modify(0.5);

    left = new EncoderFollower(modifier.getLeftTrajectory());
    right = new EncoderFollower(modifier.getRightTrajectory());

    left.configureEncoder(Robot.driveTrain.left1.getSelectedSensorPosition(), 4000, 0.1524); // 0.1524
    right.configureEncoder(Robot.driveTrain.right1.getSelectedSensorPosition(), 4000, 0.1524); // 0.1524

    System.out.println(Robot.driveTrain.left1.getSelectedSensorPosition());
    System.out.println(Robot.driveTrain.right1.getSelectedSensorPosition());

    left.configurePIDVA(0.1, 0.0, 0.0, 1 / maxVel, 0);
    right.configurePIDVA(0.1, 0.0, 0.0, 1 / maxVel, 0);

    long endTime = System.currentTimeMillis();

    System.out.println("Created tank path in ms:");
    System.out.println((endTime - startTime));

    execCount = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    leftPower = left.calculate(Robot.driveTrain.left1.getSelectedSensorPosition());
    rightPower = right.calculate(Robot.driveTrain.right1.getSelectedSensorPosition());

    // System.out.println(execCount++);
    // System.out.println(leftPower);

    double gyro_heading = Robot.sensors.getYaw();
    double desired_heading = Pathfinder.r2d(left.getHeading());

    double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
    double turn = 0.8 * (-1.0 / 80.0) * angleDifference;

    // System.out.println((System.nanoTime() - lastExecTime) / 1000000);
    // lastExecTime = System.nanoTime();

    Robot.driveTrain.setPower(leftPower + turn, rightPower - turn);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (left.isFinished() || right.isFinished()) {
      System.out.println("Done with path.");
      System.out.println(Robot.driveTrain.left1.getSelectedSensorPosition());
      System.out.println(Robot.driveTrain.right1.getSelectedSensorPosition());
      return true;
    } else {
      return false;
    }
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
