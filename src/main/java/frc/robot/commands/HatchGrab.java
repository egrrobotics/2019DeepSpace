/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;

public class HatchGrab extends CommandGroup {
  /**
   * Add your docs here.
   */
  public void RemoveHatch() {

    double bottomHeight = 100; // Where the elevator starts and returns to (encoder value).
    double topHeight = 200; // Where the elevator raises to.
    int backDistance = 10; // How far the bot backs up after raising the hatch.

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

    /*
    addSequential(new ElevatorSetHeight(topHeight));
    addSequential(new DriveForwardAuto(.5, backDistance, 0));
    addSequential(new ElevatorSetHeight(bottomHeight));
    */
  }
}