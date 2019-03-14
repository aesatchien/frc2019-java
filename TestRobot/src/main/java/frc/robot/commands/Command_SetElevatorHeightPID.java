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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Command_SetElevatorHeightPID extends Command {
  private double deltaHeight;
  Button button;
  double setpoint;
  //double kp = 0.3;
  //double ki = 0.001;
  //double kf = 0.25;
  //double kd = 0.25;
  //double kierror = 0;
  //double kderror = 0;
  double previouserror = 0;
  double error=0;
  //double tolerance = 1.0;
  //final double MAXPOWER = 0.55;
  //final double MINPOWER = 0.1;
  double counter = 0;
  final double MAXHEIGHT = 90;
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
    //Robot.elevator.setElevatorSetpoint(Robot.elevator.getElevatorHeight() +  deltaHeight);
    Robot.elevator.setElevatorSetpoint(Robot.elevator.getElevatorSetpoint() +  deltaHeight);
    previouserror = 0;
    error=0;
    //Robot.elevator.setElevatorSetpoint(deltaHeight);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
/*    double elevatorPower;
      double increment = 0.1;
      //increase the setpoint if button is held down ... why doesn't this work?
      if (!button.get()){
        Robot.elevator.setElevatorSetpoint(Robot.elevator.getElevatorSetpoint() +  Math.signum(deltaHeight)*increment);
      }
      counter = counter +1;
      error = Robot.elevator.getElevatorSetpoint()- Robot.elevator.getElevatorHeight();
      kierror = kierror + ki + error * counter;
      kderror = kd* (error-previouserror);
      elevatorPower= kf + kp*error + kierror;
      if (Robot.elevator.getElevatorSetpoint() <= 0){
        elevatorPower=0;
        Robot.elevator.setElevatorSetpoint(0);
        //Robot.elevator.setElevatorPower(0);
      }
      else{
        // make sure we never go lower than minimum power and higher than maximum power
        Robot.elevator.setElevatorPower(Math.max(MINPOWER, Math.min(MAXPOWER,elevatorPower)));
      }
      previouserror = error;
      SmartDashboard.putNumber("Elevator Error", error);
      //Timer.delay(0.05);
*/
    
double allowedDelta=10;     
double setpoint = Robot.elevator.getElevatorSetpoint() +  Math.signum(deltaHeight)*increment;
    double curatedSetpoint = Math.max(0,Math.min(MAXHEIGHT,setpoint));
    //Don't stray too much - helps in tuning
    if (curatedSetpoint > Robot.elevator.getElevatorHeight()+ allowedDelta){curatedSetpoint=Robot.elevator.getElevatorHeight()+ allowedDelta;}
    if (curatedSetpoint < Robot.elevator.getElevatorHeight() - allowedDelta){curatedSetpoint=Robot.elevator.getElevatorHeight() - allowedDelta;}
    Robot.elevator.setElevatorSetpoint((int)(100*curatedSetpoint)/100.0);
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
