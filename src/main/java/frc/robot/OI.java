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
import frc.robot.commands.IntakeRollerControl;
import frc.robot.commands.IntakeRotateControl;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  // Driver controller setup
  public Joystick driver = new Joystick(0);
  private Button driverButtonA = new JoystickButton(driver, 1);
  private Button driverButtonB = new JoystickButton(driver, 2);
  private Button driverButtonX = new JoystickButton(driver, 3);
  private Button driverButtonY = new JoystickButton(driver, 4);
  private Button driverButtonLeftBumper = new JoystickButton(driver, 5);
  private Button driverButtonRightBumper = new JoystickButton(driver, 6);
  private Button driverButtonBack = new JoystickButton(driver, 7);
  private Button driverButtonStart = new JoystickButton(driver, 8);
  private Button driverButtonLeftAxisPress = new JoystickButton(driver, 9);
  private Button driverButtonRightAxisPress = new JoystickButton(driver, 10);

  // Operator controller setup
  public Joystick operator = new Joystick(1);
  private Button operatorButtonA = new JoystickButton(operator, 1);
  private Button operatorButtonB = new JoystickButton(operator, 2);
  private Button operatorButtonX = new JoystickButton(operator, 3);
  private Button operatorButtonY = new JoystickButton(operator, 4);
  private Button operatorButtonLeftBumper = new JoystickButton(operator, 5);
  private Button operatorButtonRightBumper = new JoystickButton(operator, 6);
  private Button operatorButtonBack = new JoystickButton(operator, 7);
  private Button operatorButtonStart = new JoystickButton(operator, 8);
  private Button operatorButtonLeftAxisPress = new JoystickButton(operator, 9);
  private Button operatorButtonRightAxisPress = new JoystickButton(operator, 10);

  public OI() {
    // DriveTrain default command is ArcadeDrive
    // Elevator default command is ElevatorControl
    // Intake default command is IntakeControl 
    operatorButtonRightBumper.whileHeld(new IntakeRotateControl(-1));
    operatorButtonLeftBumper.whileHeld(new IntakeRotateControl(1));
  }

  public double getOperatorLeftTrigger() {
		return operator.getRawAxis(2);
	}

	public double getOperatorRightTrigger() {
		return operator.getRawAxis(3);
	}

	public double getOperatorTriggerDiff() {
		return getOperatorLeftTrigger() - getOperatorRightTrigger();
	}
}
