/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ElevatorControl;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {

  public TalonSRX elevatorMotor = new TalonSRX(RobotMap.elevator);
 
  public double targetHeight = 0;

  public Elevator() {}

  public void setPower(double power) {
    SmartDashboard.putNumber("Elevator power:", power);
    SmartDashboard.putNumber("Elevator target:", targetHeight);

    // Min/max
    power = Math.min(0.8, power);
    power = Math.max(-0.8, power);

    // Deadband
    if (Math.abs(power) < 0.1) {
      power = 0;
    } else if (Math.abs(power) < 0.2) {
      power = (power < 0) ? -0.2 : 0.2;
    }

    elevatorMotor.set(ControlMode.PercentOutput, power);
  }

  public void setTargetHeight(double height) {
    double min = -1000;
    double max = 1000000;
    targetHeight = Math.max(min, Math.min(max, height));
  }

  public void moveTowardTarget() {
    double error = targetHeight - Robot.sensors.getElevatorEncoder();
    double power = error * 0.00001;
    setPower(power);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new ElevatorControl());
  }
}
