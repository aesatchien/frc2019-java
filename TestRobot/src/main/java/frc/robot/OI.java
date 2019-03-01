/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.*;
import frc.robot.spartanutils.AxisButton;
/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  public Joystick stick;
  public JoystickButton buttonA;
  public JoystickButton buttonB;
  public JoystickButton buttonX;
  public JoystickButton buttonY;
  public JoystickButton buttonLB;
  public JoystickButton buttonRB;
  public JoystickButton buttonBack;
  public JoystickButton buttonStart;
  public POVButton povButtonUp;
  public POVButton povButtonDown;
  public POVButton povButtonLeft;
  public POVButton povButtonRight;
  public AxisButton axisButtonLT;
  public AxisButton axisButtonRT;
  


  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

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

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
  public OI(){
    stick = new Joystick(0);
    
    buttonA = new JoystickButton(stick, 1);
    buttonB = new JoystickButton(stick, 2);
    buttonX = new JoystickButton(stick, 3);
    buttonY = new JoystickButton(stick, 4);
    buttonLB = new JoystickButton(stick, 5);
    buttonRB = new JoystickButton(stick, 6);
    buttonBack = new JoystickButton(stick,7);
    buttonStart = new JoystickButton(stick, 8);
    povButtonUp = new  POVButton(stick, 180);
    povButtonDown = new  POVButton(stick, 0);
    povButtonRight = new  POVButton(stick, 90);
    povButtonLeft = new  POVButton(stick, 270);
    axisButtonLT = new AxisButton(stick, 2);
    axisButtonRT = new AxisButton(stick, 3);



    //buttonA.whenPressed(new Command_SetSparkMax(0.0, buttonA));
    //buttonB.whenPressed(new Command_SetSparkMax(0.1, buttonB));
    //buttonX.whenPressed(new Command_SetSparkMax(-0.1, buttonX));
    buttonA.whenPressed(new Command_SetSolenoid(0,buttonA));
    buttonY.whenPressed(new Command_SetSolenoid(2,buttonY));
    //buttonB.whenPressed(new Command_SetSolenoid(1,buttonB));
    //buttonX.whenPressed(new Command_SetSolenoid(-1,buttonX));
    buttonLB.whenPressed(new SetElevator(-0.3,buttonLB));
    buttonRB.whenPressed(new SetElevator(0.5,buttonRB));
    buttonBack.whenPressed(new Command_SetSolenoid(5,buttonBack));
    buttonStart.whenPressed(new Command_PneumaticDrive(0.5,buttonStart));
    povButtonUp.whenPressed(new SetElevator(-0.3, povButtonUp));
    povButtonDown.whenPressed(new SetElevator(0.75, povButtonDown));
    axisButtonLT.whenPressed(new SetElevator(-0.3,axisButtonLT,2));
    axisButtonRT.whenPressed(new SetElevator(0.3,axisButtonRT,3));
  }
}