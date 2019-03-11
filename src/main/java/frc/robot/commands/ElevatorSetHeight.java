/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class ElevatorSetHeight extends InstantCommand {

  double target;

  public ElevatorSetHeight(double target) {
    super();
    // requires(Robot.elevator);
    this.target = target;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.elevator.setTargetHeight(target);
  }

}
