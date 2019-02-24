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

/**
 * Add your docs here.
 */
public class Climber extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  TalonSRX climbMotor1;
  TalonSRX climbMotor2;
  TalonSRX climbMotorMaster;
  
  public Climber() {
    climbMotor1 = new TalonSRX(9);
    climbMotor2 = new TalonSRX(10);
    climbMotorMaster = new TalonSRX(11);
  }

  public void setClimbMotor1(double power) {
    climbMotor1.set(ControlMode.PercentOutput, power);
  }
  public void setClimbMotor2(double power) {
    climbMotor2.set(ControlMode.PercentOutput, power);
  }
  public void setClimbMotorMaster(double power) {
    climbMotorMaster.set(ControlMode.PercentOutput, power);
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
