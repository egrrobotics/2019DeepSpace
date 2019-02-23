/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.ArcadeDrive;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  TalonSRX left1;
  TalonSRX left2;
  TalonSRX right1;
  TalonSRX right2; 
 
  public DriveTrain() {
    left1 = new TalonSRX(RobotMap.driveLeft1);
    left2 = new TalonSRX(RobotMap.driveLeft2);
    right1 = new TalonSRX(RobotMap.driveRight1);
    right2 = new TalonSRX(RobotMap.driveRight2); 
    right1.setInverted(true);
    right2.setInverted(true);
  }

  private double deadBand(double power) {
    if(Math.abs(power) < 0.1) {
      return 0;
    }
    return power;
  }

  private void setLeftPower(double leftPower) {
    leftPower = deadBand(leftPower);
    left1.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, leftPower);
    left2.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, leftPower);
  }

  private void setRightPower(double rightPower) {
    rightPower = deadBand(rightPower);
    right1.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, rightPower);
    right2.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, rightPower);
  }

  public void setPower(double power) {
    setLeftPower(power);
    setRightPower(power);
  }
  
  public void setPower(double leftPower, double rightPower) {
    setLeftPower(leftPower);
    setRightPower(rightPower);
  }

  public int getLeftEncoder() {
    return left1.getSelectedSensorPosition();
  }

  public int getRightEncoder() {
    return right1.getSelectedSensorPosition();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    this.setDefaultCommand(new ArcadeDrive());
  }
}
