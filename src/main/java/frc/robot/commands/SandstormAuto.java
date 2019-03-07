/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class SandstormAuto extends CommandGroup {
  /**
   * Add your docs here.
   */

  public enum AutoType {
    LEFT, RIGHT
  }
  
  public SandstormAuto(AutoType type) {
    switch(type) {
      case LEFT:
        addSequential(new DriveForwardAuto(0.5, 47, 0));
        addSequential(new RotateToHeading(25, 0.5));
        addSequential(new DriveForwardAuto(0.5, 183, 25));
        addSequential(new RotateToHeading(155, 0.5));
        addSequential(new DriveForwardAuto(0.5, 24, 155));
        addSequential(new ElevatorSetHeight(1700));
        addSequential(new WaitCommand(2.0));
        addSequential(new ElevatorSetHeight(0));
        addSequential(new WaitCommand(1.8));
        addSequential(new IntakePop(), 0.2);
        addSequential(new DriveBackwardAuto(0.5, 35, 155));
        addSequential(new RotateToHeading(-160, 0.5));
        addSequential(new DriveForwardAuto(0.5, 100, -160));
        addSequential(new RotateToHeading(165, 0.5));
        addSequential(new DriveForwardAuto(0.5, 130, 165));
        break;
      case RIGHT:
        addSequential(new DriveForwardAuto(0.5, 47, 0));
        addSequential(new RotateToHeading(-25, 0.5));
        addSequential(new DriveForwardAuto(0.5, 183, -25));
        addSequential(new RotateToHeading(-155, 0.5));
        addSequential(new DriveForwardAuto(0.5, 24, -155));
        addSequential(new ElevatorSetHeight(1700));
        addSequential(new WaitCommand(2.0));
        addSequential(new ElevatorSetHeight(0));
        addSequential(new WaitCommand(1.8));
        addSequential(new IntakePop(), 0.2);
        addSequential(new DriveBackwardAuto(0.5, 35, -155));
        addSequential(new RotateToHeading(160, 0.5));
        addSequential(new DriveForwardAuto(0.5, 100, 160));
        addSequential(new RotateToHeading(-165, 0.5));
        addSequential(new DriveForwardAuto(0.5, 130, -165));
        break;
    }
  }
}
