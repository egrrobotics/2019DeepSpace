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
  public SandstormAuto() {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.

    addSequential(new DriveForwardAuto(0.5, 47, 0));
    addSequential(new RotateToHeading(25, 0.5));
    addSequential(new DriveForwardAuto(0.5, 183, 25));
    addSequential(new RotateToHeading(155, 0.5));
    addSequential(new ElevatorSetHeight(1700));
    addSequential(new DriveForwardAuto(0.5, 24, 155));
    addSequential(new WaitCommand(0.5));
    addSequential(new ElevatorSetHeight(0));
    addSequential(new WaitCommand(0.5));
    addSequential(new DriveBackwardAuto(0.5, 35, 155));
    addSequential(new RotateToHeading(-160, 0.5));
    addSequential(new DriveForwardAuto(0.5, 100, -160));
    addSequential(new RotateToHeading(165, 0.5));
    addSequential(new DriveForwardAuto(0.5, 130, 165));
  }
}
