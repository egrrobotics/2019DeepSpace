/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Pneumatics extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public Compressor compressor = new Compressor(RobotMap.PCM);

  Solenoid intakePop = new Solenoid(RobotMap.PCM, RobotMap.popperSolenoid);

  Solenoid hatchGrabber = new Solenoid(RobotMap.PCM, RobotMap.hatchGrabberSolenoid);
  boolean hatchGrabberOpen = false;

  Solenoid intakeExtender = new Solenoid(RobotMap.PCM, RobotMap.intakeExtenderSolenoid);
  boolean intakeExtenderOpen = false;

  public Pneumatics() {
    compressor.start();
    // compressor.stop();
  }

  public void setIntakePop(boolean extended) {
    intakePop.set(extended);
  }

  public void toggleHatchGrabber() {
    hatchGrabberOpen = !hatchGrabberOpen;
    hatchGrabber.set(hatchGrabberOpen);
  }

  public void toggleIntakeExtended() {
    intakeExtenderOpen = !intakeExtenderOpen;
    intakeExtender.set(intakeExtenderOpen);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
