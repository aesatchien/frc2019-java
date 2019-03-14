/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.pneumatics;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.buttons.*;

public class Command_Shifters extends Command {
  int state;
  JoystickButton button;
  
  public Command_Shifters() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.pneumatics);
  }
  public Command_Shifters(int state, JoystickButton button) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this();
    this.state = state;
    this.button = button;
  }
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if (state ==0) {Robot.pneumatics.lowGear();}
    if (state ==1) {Robot.pneumatics.highGear();}
        
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

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
        System.out.println("\nEnded "+  this.getClass().getSimpleName() +"("+ String.format("%d",this.state) +") at " + String.format("%.2f",(Timer.getFPGATimestamp()-Robot.enabledTime)) + "s");
        //Robot.pneumatics.solenoidOff();
      }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
