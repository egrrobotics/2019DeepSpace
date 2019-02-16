/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.ElevatorControl;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {

  TalonSRX elevatorMotor = new TalonSRX(6);
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public Elevator() {}

  public void setPower(double power) {
    elevatorMotor.set(ControlMode.PercentOutput, power);
  }

  public double getCurrentHeight() {
    return elevatorMotor.getSelectedSensorPosition();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new ElevatorControl());
  }
}
