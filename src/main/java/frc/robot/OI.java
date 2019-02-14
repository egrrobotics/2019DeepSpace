/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// WOAH DUDE IT"S CODE

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.ElevatorControl;
import frc.robot.commands.IntakeRollerControl;
import frc.robot.commands.IntakeRotateControl;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  public Joystick driver = new Joystick(0);
  public Joystick operator = new Joystick(1);

  public OI() {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);
   Button rightOpBumper = new JoystickButton(operator, RobotMap.rightBumper);
   Button leftOpBumper = new JoystickButton(operator, RobotMap.leftBumper);
   Button opAButton = new JoystickButton(operator, RobotMap.buttonA);
   Button opBButton = new JoystickButton(operator, RobotMap.buttonB);
   Button opXButton = new JoystickButton(operator, RobotMap.buttonX);
   Button opYButton = new JoystickButton(operator, RobotMap.buttonY);
  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());
    rightOpBumper.whileHeld(new ElevatorControl(1));
    leftOpBumper.whileHeld(new ElevatorControl(-1));
    opAButton.whileHeld(new IntakeRollerControl(1));
    opBButton.whileHeld(new IntakeRollerControl(-1));
    opXButton.whileHeld(new IntakeRotateControl(1));
    opYButton.whileHeld(new IntakeRotateControl(-1));
  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
  }
}
