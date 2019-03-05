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
import java.text.DecimalFormat;

import javax.naming.InitialContext;

import frc.robot.spartanutils.BNO055;

public class Command_RaiseRobot extends Command {
  JoystickButton button;
  private DecimalFormat f = new DecimalFormat("+000.000;-000.000");
	private double[] pos = new double[3]; // [x,y,z] position data
  private BNO055.CalData cal;
  private double initTime;
  private boolean bisInitialized;

  public Command_RaiseRobot() {
    requires(Robot.pneumatics);
  }

  public Command_RaiseRobot(JoystickButton button) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this();
    this.button = button;
    this.initTime = Timer.getFPGATimestamp();
    this.bisInitialized = false;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    double now = Timer.getFPGATimestamp();
    System.out.println("\nInitialized "+  this.getClass().getSimpleName() +"() at " + String.format("%.2f",(Timer.getFPGATimestamp()-Robot.enabledTime)) + "s");
   
    //Need to find a way to know if we have been reset for testing...
    // Only reset the gyro if it has been minutes since the last reset
    if (bisInitialized){
      if (now - initTime > 120) {
        Robot.drivetrain.driveGyro.reset();
        initTime = now;
    }
    else {
      bisInitialized = true;
      Robot.drivetrain.driveGyro.reset();
      initTime = now;
    }
    

    }
    // System.out.println("COMMS: " + Robot.drivetrain.isSensorPresent()
    // + ", INITIALIZED: " + Robot.drivetrain.isInitialized()
    // + ", CALIBRATED: " + Robot.drivetrain.isCalibrated());

    //  /* Display calibration status for each sensor. */
    //  cal = Robot.drivetrain.getCalibration();
    //  System.out.println("\tCALIBRATION: Sys=" + cal.sys
    //   + " Gyro=" + cal.gyro + " Accel=" + cal.accel
    //   + " Mag=" + cal.mag);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  //   if(Robot.drivetrain.isInitialized()){
  //     pos = Robot.drivetrain.getVector();

  //     /* Display the floating point data */
  //     System.out.println("\tX: " + f.format(pos[0])
  //     + " Y: " + f.format(pos[1]) + " Z: " + f.format(pos[2])
  //     + "  H: " + Robot.drivetrain.getHeading());
  // }
    Robot.pneumatics.raiseRobot();
    Timer.delay(0.1);
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
