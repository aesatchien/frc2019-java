/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.buttons.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Command_SetElevatorHeightPID extends Command {
  private double deltaHeight;
  Button button;
  double setpoint;
  double counter = 0;
  final double MAXHEIGHT = 60;
  double increment = 0.3;

  public Command_SetElevatorHeightPID() {
    // Use requires() here to declare subsystem dependencies
    //requires(Robot.elevator);
  }

  public Command_SetElevatorHeightPID(double deltaHeight) {
    // Use requires() here to declare subsystem dependencies
   this();
   this.deltaHeight = deltaHeight;
  }
  
  public Command_SetElevatorHeightPID(double deltaHeight,Button button) {
    // Use requires() here to declare subsystem dependencies
   this();
   this.deltaHeight = deltaHeight;
   this.button = button;
   this.increment = 0.3;
  }
  public Command_SetElevatorHeightPID(double deltaHeight, double increment, Button button) {
    // Use requires() here to declare subsystem dependencies
   this();
   this.deltaHeight = deltaHeight;
   this.button = button;
   this.increment = increment;
  }



  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("\nStarted "+  this.getClass().getSimpleName() +"("+ String.format("%.1f",this.deltaHeight) +") and button value: "+ Robot.oi.stick.getPOV(0) +" at " + String.format("%.2f",(Timer.getFPGATimestamp()-Robot.enabledTime)) + "s");
    Robot.elevator.setElevatorSetpoint(Robot.elevator.getElevatorSetpoint() +  deltaHeight);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  
    double allowedDelta=10;     
    double setpoint = Robot.elevator.getElevatorSetpoint() +  Math.signum(deltaHeight)*increment;
    //Don't stray too much - helps in tuning
    if (setpoint > Robot.elevator.getElevatorHeight() + allowedDelta){setpoint = Robot.elevator.getElevatorHeight() + allowedDelta;}
    if (setpoint < Robot.elevator.getElevatorHeight() - allowedDelta){setpoint = Robot.elevator.getElevatorHeight() - allowedDelta;}
    double curatedSetpoint = Math.max(0,Math.min(MAXHEIGHT,setpoint));
    Robot.elevator.setElevatorSetpoint(curatedSetpoint);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    //return (Math.abs(error) < tolerance);
    return !(button.get());
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    //Robot.elevator.setElevatorPower(0);
    System.out.println("\nEnded "+  this.getClass().getSimpleName() +"("+ String.format("%.1f",this.deltaHeight) +") at " + String.format("%.2f",(Timer.getFPGATimestamp()-Robot.enabledTime)) + "s");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    //Robot.elevator.setElevatorPower(0);
    System.out.println("\nInterrupted "+  this.getClass().getSimpleName() +"("+ String.format("%.1f",this.deltaHeight) +") at " + String.format("%.2f",(Timer.getFPGATimestamp()-Robot.enabledTime)) + "s");
  }
}
