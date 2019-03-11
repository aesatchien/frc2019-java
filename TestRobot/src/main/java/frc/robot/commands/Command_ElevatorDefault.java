/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Command_ElevatorDefault extends Command {

  double setpoint;
  double kp = 0.25;
  double kf = 0.1;
  double error=0;
  double tolerance = 1.0;
  double previouserror = 0;
  final double MAXPOWER = 0.75;
  final double MINPOWER = 0.-30;


  public Command_ElevatorDefault() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.elevator);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("\nStarted "+  this.getClass().getSimpleName() + " at " + String.format("%.2f",(Timer.getFPGATimestamp()-Robot.enabledTime)) + "s");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //if (Robot.elevator.isElevatorLow()) {
    //  Robot.elevator.setElevatorPower(0.0);      
    //} else {
      //Do nothing, because the new set height command will take over
      //Robot.elevator.setElevatorPower(0.5);
    //}
    double elevatorPower;
      error = Robot.elevator.getElevatorSetpoint()- Robot.elevator.getElevatorHeight();
      elevatorPower= kf + kp*error;
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

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
