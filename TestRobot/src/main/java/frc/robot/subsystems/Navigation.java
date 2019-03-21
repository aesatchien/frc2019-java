/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;
import edu.wpi.first.wpilibj.DriverStation; 
import edu.wpi.first.wpilibj.SerialPort;
/**
 * Add your docs here.
 */
public class Navigation extends Subsystem {
  private static AHRS ahrs;
  int counter = 0;

  //Front high is roll negative, Left high is pitch negative
  //Yaw is the heading
 
  public Navigation() {
    super();
    try {
      ahrs = new AHRS(SerialPort.Port.kMXP, SerialDataType.kProcessedData, (byte)50);
      ahrs.enableLogging(true);
    } 
    catch (RuntimeException ex ) {
      DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }
  
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public double getTilt(){
    return ahrs.getRoll();
  }
  public double getYaw(){
    return  ahrs.getYaw();
  }
  public double getPitch(){
    return ahrs.getPitch();
  }
    

  public void log(){
    counter ++;
    if (Math.floorMod(counter, 10) == 0) {
      SmartDashboard.putBoolean(  "IMU_Connected",        ahrs.isConnected());
      SmartDashboard.putBoolean(  "IMU_IsCalibrating",    ahrs.isCalibrating());
      SmartDashboard.putNumber(   "IMU_Yaw",              ahrs.getYaw());
      SmartDashboard.putNumber(   "IMU_Pitch",            ahrs.getPitch());
      SmartDashboard.putNumber(   "IMU_Roll",             ahrs.getRoll());
      
      /* Display tilt-corrected, Magnetometer-based heading (requires             */
      /* magnetometer calibration to be useful)                                   */
      SmartDashboard.putNumber(   "IMU_CompassHeading",   ahrs.getCompassHeading());
      /* Display 9-axis Heading (requires magnetometer calibration to be useful)  */
      //SmartDashboard.putNumber(   "IMU_FusedHeading",     ahrs.getFusedHeading());
      /* These functions are compatible w/the WPI Gyro Class, providing a simple  */
      /* path for upgrading from the Kit-of-Parts gyro to the navx MXP            */
      SmartDashboard.putNumber(   "IMU_TotalYaw",         ahrs.getAngle());
      //SmartDashboard.putNumber(   "IMU_YawRateDPS",       ahrs.getRate());
      /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */
      //SmartDashboard.putNumber(   "IMU_Accel_X",          ahrs.getWorldLinearAccelX());
      //SmartDashboard.putNumber(   "IMU_Accel_Y",          ahrs.getWorldLinearAccelY());
      //SmartDashboard.putBoolean(  "IMU_IsMoving",         ahrs.isMoving());
      //SmartDashboard.putBoolean(  "IMU_IsRotating",       ahrs.isRotating());
    }
  }
}
