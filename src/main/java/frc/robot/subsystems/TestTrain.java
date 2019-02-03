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
import frc.robot.commands.TestDrive;

// ALL THIS CODE WAS WRITTEN BY BEN
// IF THERE IS A PROBLEM, TELL HIM

/**
 * Add your docs here.
 */
public class TestTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  TalonSRX talon = new TalonSRX(1); // Testing Talon (Can ID of 1).
  ControlMode talonMode = com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput; // Talon control mode.

  public void setPower(double power) {
    power = limitPower(power, .1, .8); // Deadband power at .1, clip power at .8.
    talon.set(talonMode, power); // Set the power of the testing Talon.
  }

  private double limitPower(double power, double deadBand, double clip){
    // Deadbander:
    if (Math.abs(power)<deadBand){
      power = 0;
    }
    // Clipper:
    if (Math.abs(power)>clip){
      if (power>0){
        power = clip;
      } else if (power<0){
        power = -clip;
      }
    }
    // Returner:
    return power;
  } 

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    this.setDefaultCommand(new TestDrive());
  }
}
