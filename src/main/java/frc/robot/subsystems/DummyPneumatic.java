/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.hal.CompressorJNI;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */

public class DummyPneumatic extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  DoubleSolenoid dummyDouble = new DoubleSolenoid(0, 1);
  public Compressor compressor = new Compressor(0);

  public DummyPneumatic() {
    dummyDouble.set(DoubleSolenoid.Value.kOff);

    System.out.println("Set up compressor.");
    // compressor.setClosedLoopControl(true);
    compressor.start();
    System.out.println(compressor.enabled());
    System.out.println(compressor.getPressureSwitchValue());
    System.out.println(compressor.getCompressorCurrent());
  }

  public void extend() {
    dummyDouble.set(DoubleSolenoid.Value.kForward);
  }

  public void stop() {
    dummyDouble.set(DoubleSolenoid.Value.kOff);
  }

  public void reverse() {
    dummyDouble.set(DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}