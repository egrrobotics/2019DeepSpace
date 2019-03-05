/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.DriveToTarget;
import frc.robot.commands.ElevatorSetHeight;
import frc.robot.commands.HatchGrabberToggle;
import frc.robot.commands.IntakeExtenderToggle;
import frc.robot.commands.IntakePop;
import frc.robot.commands.IntakeRollerControl;
import frc.robot.commands.IntakeRotateControl;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  // Driver xbox controller
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

  // Operator xbox controller
  public Joystick operator = new Joystick(3);
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

  // Operator button board (two controllers 1)
  public Joystick buttonBoardPart1 = new Joystick(1);
  private Button buttonBoard1 = new JoystickButton(buttonBoardPart1, 1);
  private Button buttonBoard2 = new JoystickButton(buttonBoardPart1, 2);
  private Button buttonBoard3 = new JoystickButton(buttonBoardPart1, 3);
  private Button buttonBoard4 = new JoystickButton(buttonBoardPart1, 4);
  private Button buttonBoard5 = new JoystickButton(buttonBoardPart1, 5);
  private Button buttonBoard6 = new JoystickButton(buttonBoardPart1, 6);
  private Button buttonBoard7 = new JoystickButton(buttonBoardPart1, 7);
  private Button buttonBoard8 = new JoystickButton(buttonBoardPart1, 8);
  private Button buttonBoard9 = new JoystickButton(buttonBoardPart1, 9);
  private Button buttonBoard10 = new JoystickButton(buttonBoardPart1, 10);
  private Button buttonBoard11 = new JoystickButton(buttonBoardPart1, 11);
  private Button buttonBoard12 = new JoystickButton(buttonBoardPart1, 12);

  public Joystick buttonBoardPart2 = new Joystick(2);
  private Button buttonBoard13 = new JoystickButton(buttonBoardPart2, 1);
  private Button buttonBoard14 = new JoystickButton(buttonBoardPart2, 2);
  private Button buttonBoard15 = new JoystickButton(buttonBoardPart2, 3);

  public OI() {
    // DriveTrain default command is ArcadeDrive
    // Elevator default command is ElevatorControl

    // Hatch presets
    buttonBoard1.whenPressed(new ElevatorSetHeight(415));
    buttonBoard2.whenPressed(new ElevatorSetHeight(1700));
    buttonBoard3.whenPressed(new ElevatorSetHeight(2930));

    // Ball presets
    buttonBoard4.whenPressed(new ElevatorSetHeight(615));
    buttonBoard5.whenPressed(new ElevatorSetHeight(1900));
    buttonBoard6.whenPressed(new ElevatorSetHeight(3130));

    // Ground, cargo bay presets
    buttonBoard7.whenPressed(new ElevatorSetHeight(-50));
    buttonBoard8.whenPressed(new ElevatorSetHeight(1500));

    // Elevator manual
    buttonBoard9.whileHeld(new ElevatorSetHeight(true));
    buttonBoard10.whileHeld(new ElevatorSetHeight(false));

    // Intake open/close
    buttonBoard11.whileHeld(new IntakeRollerControl(-1));
    buttonBoard12.whileHeld(new IntakeRollerControl(1));

    // Intake roller
    buttonBoard13.whileHeld(new IntakeRotateControl(-1));
    buttonBoard14.whileHeld(new IntakeRotateControl(1));

    // Climb
    buttonBoard15.whenPressed();

    // Driver intake controls
    driverButtonRightBumper.whileHeld(new IntakePop());
    driverButtonLeftBumper.whenPressed(new IntakeExtenderToggle());
    driverButtonB.whenPressed(new HatchGrabberToggle());

    // Driver vision
    driverButtonA.whileHeld(new DriveToTarget());
  }
  
  public void setDriverRumble(boolean on) {
    if (on) {
      driver.setRumble(RumbleType.kRightRumble, 0.5);
    } else {
      driver.setRumble(RumbleType.kRightRumble, 0);
    }
  }
}
