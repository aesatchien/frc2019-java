/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import javax.swing.text.StyleContext.SmallAttributeSet;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.spartanutils.BNO055;
import frc.robot.spartanutils.BNO055.CalData;
import frc.robot.spartanutils.BNO055.reg_t;


/**
 * Add your docs here.
 */
public class Drivetrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private final CANSparkMax sparkNeoL1 = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax sparkNeoL2 = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax sparkNeoR3 = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax sparkNeoR4 = new CANSparkMax(4, MotorType.kBrushless);
  private final SpeedControllerGroup speedGroupLeft = new SpeedControllerGroup(sparkNeoL1, sparkNeoL2);
  private final SpeedControllerGroup speedGroupRight = new SpeedControllerGroup(sparkNeoR3, sparkNeoR4);
  private final DifferentialDrive  differentialDrive = new DifferentialDrive(speedGroupLeft, speedGroupRight);
  private final CANEncoder SparkNeoEncoder1 = sparkNeoL1.getEncoder();
  private final CANEncoder SparkNeoEncoder2 = sparkNeoL2.getEncoder();
  private final CANEncoder SparkNeoEncoder3 = sparkNeoR3.getEncoder();
  private final CANEncoder SparkNeoEncoder4 = sparkNeoR4.getEncoder();
  private static BNO055 imu;
  public final Gyro driveGyro = new ADXRS450_Gyro();
  private final double twistSensitivity = -0.5;
  private int counter;

  public Drivetrain() {
    super();
    imu = BNO055.getInstance(BNO055.opmode_t.OPERATION_MODE_IMUPLUS,
      BNO055.vector_type_t.VECTOR_EULER);
  
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new DriveByJoystick());
  }
  public void SparkWithStick(double xSpeed, double zRotation) { 	
    
    differentialDrive.arcadeDrive(xSpeed, twistSensitivity*zRotation, false);
    if (xSpeed >0.5){
       //Digital0.set(true);
    }
    //else {Digital0.set(false);}
  }

  public void reset(){
    //sparkNeoL1.set(0);
  }

  //--- BEGIN IMPORTS OF IMU FUNCTIONS FROM TEAM 2168 GIT PROJECT
  /**
	 * The heading of the sensor (x axis) in continuous format. Eg rotating the
	 *   sensor clockwise two full rotations will return a value of 720 degrees.
	 *
	 * @return heading in degrees
     */
    public double getHeading() {
    	return imu.getHeading();
    }
    
    /**
     * Gets a vector representing the sensors position (heading, roll, pitch).
	 * heading:    0 to 360 degrees
	 * roll:     -90 to +90 degrees
	 * pitch:   -180 to +180 degrees
	 *
	 * For continuous rotation heading (doesn't roll over between 360/0) see
	 *   the getHeading() method.
	 *
	 * @return a vector [heading, roll, pitch]
	 */
    public double[] getVector() {
    	return imu.getVector();
    }
    
	/**
	 * @return true if the IMU is found on the I2C bus
	 */
	public boolean isSensorPresent() {
		return imu.isSensorPresent();
	}

	/** 
	 * @return true when the IMU is initialized.
	 */
	public boolean isInitialized() {
		return imu.isInitialized();
	}
	
	/**
	 * Gets current IMU calibration state.
	 * @return each value will be set to 0 if not calibrated, 3 if fully
	 *   calibrated.
	 */
	public BNO055.CalData getCalibration() {
		return imu.getCalibration();
	}
	
	/**
	 * Returns true if all required sensors (accelerometer, magnetometer,
	 *   gyroscope) in the IMU have completed their respective calibration
	 *   sequence.
	 * @return true if calibration is complete for all sensors required for the
	 *   mode the sensor is currently operating in. 
	 */
	public boolean isCalibrated() {
		return imu.isCalibrated();
	}

  public void log() {
    counter ++;
    if (Math.floorMod(counter, 10) == 0) {
      SmartDashboard.putNumber("Position Enc1", 0.1*Math.round(10*SparkNeoEncoder1.getPosition()));
      SmartDashboard.putNumber("Velocity Enc1", 0.1*Math.round(10*SparkNeoEncoder1.getVelocity()));
      SmartDashboard.putNumber("Position Enc2", 0.1*Math.round(10*SparkNeoEncoder2.getPosition()));
      SmartDashboard.putNumber("Position Enc3", 0.1*Math.round(10*SparkNeoEncoder3.getPosition()));
      SmartDashboard.putNumber("Velocity Enc3", 0.1*Math.round(10*SparkNeoEncoder3.getVelocity()));
      SmartDashboard.putNumber("Position Enc4", Math.round(SparkNeoEncoder4.getPosition()));
      SmartDashboard.putNumber("DriveGyro", Math.round(driveGyro.getAngle()));
    }
  }
}
