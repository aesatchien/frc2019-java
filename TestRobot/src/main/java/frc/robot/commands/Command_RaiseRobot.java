/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.buttons.*;

public class Command_RaiseRobot extends Command {
  JoystickButton button;

  public Command_RaiseRobot() {
    requires(Robot.pneumatics);
  }

  public Command_RaiseRobot(JoystickButton button) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this();
    this.button = button;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //Robot.pneumatics.set_tiltOffset();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.pneumatics.raiseRobot();
    Timer.delay(0.15);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return ! button.get();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    System.out.println("\nEnded "+  this.getClass().getSimpleName() +"() at " + String.format("%.2f",(Timer.getFPGATimestamp()-Robot.enabledTime)) + "s");
    Robot.pneumatics.solenoidOff();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    System.out.println("\nInterrupted "+  this.getClass().getSimpleName() +"() at " + String.format("%.2f",(Timer.getFPGATimestamp()-Robot.enabledTime)) + "s");
    Robot.pneumatics.solenoidOff();
  }
}
