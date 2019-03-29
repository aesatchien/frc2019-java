/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.pneumatics;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.buttons.*;
import edu.wpi.first.wpilibj.Timer;

public class Command_PneumaticDrive extends Command {
  double speed = 0;
  JoystickButton button;
  public Command_PneumaticDrive() {
    requires(Robot.drivetrain);
  }
  
  public Command_PneumaticDrive(double myspeed, JoystickButton mybutton) {
    this();
    this.speed = myspeed;
    this.button = mybutton;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("\nStarted "+  this.getClass().getSimpleName() + "at " + String.format("%.2f",(Timer.getFPGATimestamp()-Robot.enabledTime)) + "s");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.pneumatics.pneumaticDrive(speed);
    //Robot.drivetrain.smoothDrive(-0.2, 0);
    Robot.drivetrain.setVelocity(100);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return ! button.get();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    System.out.println("\nEnded "+  this.getClass().getSimpleName() + "at " + String.format("%.2f",(Timer.getFPGATimestamp()-Robot.enabledTime)) + "s");
    Robot.pneumatics.pneumaticDrive(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    System.out.println("\nInterrupted "+  this.getClass().getSimpleName() + "at " + String.format("%.2f",(Timer.getFPGATimestamp()-Robot.enabledTime)) + "s");
    Robot.pneumatics.pneumaticDrive(0);
  }
}
