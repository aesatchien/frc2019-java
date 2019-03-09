/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.buttons.*;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.spartanutils.AxisButton;

/**
 * An example command.  You can replace me with your own command.
 */
public class Command_SetElevator extends Command {
  private double speed;
  int axis;
  Button button;
  boolean bAxisButton; 
  public Command_SetElevator() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.elevator);
  }

  public Command_SetElevator(double speed) {
    // Use requires() here to declare subsystem dependencies
   this();
   this.speed = speed;
  }
  
  public Command_SetElevator(double speed,Button button) {
    // Use requires() here to declare subsystem dependencies
   this();
   this.speed = speed;
   this.button = button;
   this.bAxisButton = false;
   axis=0;
  }

  public Command_SetElevator(double speed, AxisButton button, int axis) {
    // Use requires() here to declare subsystem dependencies
   this();
   this.speed = speed;
   this.button = button;
   this.bAxisButton = true;
   this.axis = axis;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("\nStarted "+  this.getClass().getSimpleName() +"("+ String.format("%.1f",this.speed) +") at " + String.format("%.2f",(Timer.getFPGATimestamp()-Robot.enabledTime)) + "s");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (bAxisButton){
      if(axis==3){speed=Robot.oi.stick.getRawAxis(axis);}
      if(axis==2){speed=-0.5*Robot.oi.stick.getRawAxis(axis);}
    }
    Robot.elevator.setElevatorSpeed(speed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return ! button.get();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.elevator.setElevatorSpeed(0);
    System.out.println("\nEnded "+  this.getClass().getSimpleName() +"("+ String.format("%.1f",this.speed) +") at " + String.format("%.2f",(Timer.getFPGATimestamp()-Robot.enabledTime)) + "s");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.elevator.setElevatorSpeed(0);
    System.out.println("\nInterrupted "+  this.getClass().getSimpleName() +"("+ String.format("%.1f",this.speed) +") at " + String.format("%.2f",(Timer.getFPGATimestamp()-Robot.enabledTime)) + "s");
  }
}
