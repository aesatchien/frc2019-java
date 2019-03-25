/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.hal.sim.mockdata.RoboRioDataJNI;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.Timer;


public class Command_AutoRotate extends Command {
  double setpoint;
  double tolerence = 3.0;
  double kp = 0.01; 
  double kd= 0.1;
  double kf = 0.3;
  double startYaw, error, power, prevError;

  private Command_AutoRotate() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.drivetrain);
    this.setTimeout(3);
  }

  public Command_AutoRotate(double setpoint) {
    // Use requires() here to declare subsystem dependencies
    this();
    this.setpoint = setpoint;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("\nStarted "+  this.getClass().getSimpleName() +"("+ String.format("%.1f",this.setpoint) +") and button value: "+ Robot.oi.stick.getPOV(0) +" at " + String.format("%.2f",(Timer.getFPGATimestamp()-Robot.enabledTime)) + "s");
    Robot.drivetrain.goToSetPoint(setpoint);
    startYaw = Robot.navigation.getYaw();
    error = 0;
    prevError = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {   
    error = setpoint - (Robot.navigation.getYaw()-startYaw);
    power=kp*error+ kf*Math.signum(error)+kd*(error-prevError)/0.02;
    prevError=error;
    Robot.drivetrain.tankDrive(-power,+power);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Math.abs(error)<=tolerence || this.isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    System.out.println("\nEnded "+  this.getClass().getSimpleName() +"("+ String.format("%.1f",this.setpoint) + " at " + String.format("%.2f",(Timer.getFPGATimestamp()-Robot.enabledTime)) + "s");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    System.out.println("\nInterrupted "+  this.getClass().getSimpleName() +"("+ String.format("%.1f",this.setpoint) + " at " + String.format("%.2f",(Timer.getFPGATimestamp()-Robot.enabledTime)) + "s");
  }
}
