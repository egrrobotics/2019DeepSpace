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

  public TalonSRX elevatorMotor = new TalonSRX(6);
  public double targetHeight = 0;

  public Elevator() {}

  public void setPower(double power) {
    // Min/max
    power = Math.min(1.0, power);
    power = Math.max(-1.0, power);

    // Deadband
    if (Math.abs(power) < 0.1) {
      power = 0;
    } else if (Math.abs(power) < 0.2) {
      power = (power < 0) ? -0.2 : 0.2;
    }

    elevatorMotor.set(ControlMode.PercentOutput, -power);
  }

  public double getHeight() {
    return -elevatorMotor.getSelectedSensorPosition();
  }

  public void setTargetHeight(double height) {
    double min = -500;
    double max = 3200;
    targetHeight = Math.max(min, Math.min(max, height));
  }

  public void moveTowardTarget() {
    double error = targetHeight - getHeight();
    double power = error * 0.005;
    setPower(power);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new ElevatorControl());
  }
}
