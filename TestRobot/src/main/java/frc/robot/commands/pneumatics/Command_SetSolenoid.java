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

public class Command_SetSolenoid extends Command {
  String state;
  JoystickButton button;
  boolean bButtonless;

  public Command_SetSolenoid() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.pneumatics);
  }

  public Command_SetSolenoid(String state) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this();
    this.state = state;
    bButtonless = true;
    this.button = button;
  }
  public Command_SetSolenoid(String state, JoystickButton button) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this();
    this.state = state;
    this.button = button;
    bButtonless = false;
  }
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if (state.equals("climb")) {
      Robot.pneumatics.toggleClimbingEnabled();
      Robot.pneumatics.setPitchOffset();
      Robot.pneumatics.setTiltOffset();
    }
    if (state.equals("compressor")) {Robot.pneumatics.compressorToggle();}
    if (state.equals("hatch")) {Robot.pneumatics.hatchToggle();}
    if (state.equals("retractboth")) {Robot.pneumatics.retractFrontAndBack();}
    if (state.equals("float")) {Robot.pneumatics.solenoidOff();}
    if (state.equals("togglegear")) {Robot.pneumatics.gearToggle();}      
    if (Robot.pneumatics.isClimbingEnabled()){
      if (state.equals("retractfront")) {Robot.pneumatics.retractFront();;}
      if (state.equals("retractback")) {Robot.pneumatics.retractBack();;}      
      //if (state ==1) {Robot.pneumatics.raiseRobot();}
       
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    //if (bButtonless){return true;}
    //else{return ! button.get();}
    return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    //System.out.println("\nAnalog 0 is " + String.format("%.2f",Robot.drivetrain.getAnalog0()));
        System.out.println("\nEnded "+  this.getClass().getSimpleName() +"("+ state +") at " + String.format("%.2f",(Timer.getFPGATimestamp()-Robot.enabledTime)) + "s");
        //Robot.pneumatics.solenoidOff();
      }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
