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

public class Command_SetSparkMax extends Command {
  double speed;
  JoystickButton button;
  public Command_SetSparkMax() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.drivetrain);
  }
  public Command_SetSparkMax(double vel, JoystickButton button) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this();
    this.speed = vel;
    this.button = button;
  }
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
   
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.drivetrain.SparkWithStick(speed,0);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    
    return ! button.get();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    //System.out.println("\nAnalog 0 is " + String.format("%.2f",Robot.drivetrain.getAnalog0()));
    System.out.println("\nEnded "+  this.getClass().getSimpleName() +"("+ String.format("%.1f",this.speed) +") at " + String.format("%.2f",(Timer.getFPGATimestamp()-Robot.enabledTime)) + "s");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
