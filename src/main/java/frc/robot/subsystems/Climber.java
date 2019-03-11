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
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Climber extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public TalonSRX verticalMotor = new TalonSRX(RobotMap.climberVertical);
  public TalonSRX horizontalMotor = new TalonSRX(RobotMap.climberHorizontal);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  private double restrictPower(double power) {
    // Min/max
    power = Math.min(0.5, power);
    power = Math.max(-0.5, power);

    // Deadband
    if (Math.abs(power) < 0.1) {
      power = 0;
    } else if (Math.abs(power) < 0.2) {
      power = (power < 0) ? -0.2 : 0.2;
    }

    return power;
  }

  public void setVerticalPower(double power) {
    verticalMotor.set(ControlMode.PercentOutput, restrictPower(power));
  }

  public void setHorizontalPower(double power) {
    horizontalMotor.set(ControlMode.PercentOutput, restrictPower(power));
  }
}
