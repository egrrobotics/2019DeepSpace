/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
  public static double encoderCountsPerInch = 222;

  // Drivetrain Talons
  public static int driveRight1 = 3;
  public static int driveRight2 = 4;
  public static int driveLeft1 = 5;
  public static int driveLeft2 = 6;

  // Elevator Talons
  public static int elevator = 7;
  public static int elevatorEncoderA = 0;
  public static int elevatorEncoderB = 1;

  // Intake Talons
  public static int intakeRotate = 8;
  public static int intakeRoller = 9;

  // Climber talons
  public static int climberVertical = 11;
  public static int climberHorizontal = 10;

  // Pneumatic IDs
  public static int PCM = 0;
  public static int popperSolenoid = 0;
  public static int hatchGrabberSolenoid = 2;
  public static int intakeExtenderSolenoid = 1;
}
