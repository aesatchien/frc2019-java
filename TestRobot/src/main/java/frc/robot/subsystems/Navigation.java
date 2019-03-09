/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.spartanutils.BNO055;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
/**
 * Add your docs here.
 */
public class Navigation extends Subsystem {
  private static BNO055 imu;
  private double badTiltValue = 0;
  int counter = 0;
  private double pitch;
  private double yaw;
  private double tiltOffset = 0;
  private int badTilts = 0;
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
 
  public Navigation() {
    super();
    imu = BNO055.getInstance(BNO055.opmode_t.OPERATION_MODE_IMUPLUS,
      BNO055.vector_type_t.VECTOR_EULER);
  
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
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
  
  // IMU keeps giving a bad value that messes up the raiseRobot routine
  // Try to find two bad values and then save it
  public double findBadValue() {
    System.out.println("Starting to look for bad values...");
    double badValue = 0;
    double[] pos = {0,0,0};
    double[] oldpos = {0,0,0};
    int i=0;
    double tolerance = 0.001;
    boolean bFoundBadValue = false;

    oldpos = getVector();
    System.out.println("Initial value: "+  oldpos[1]);
    for (i = 0; i < 1000; i++){
      pos= getVector();
      System.out.println("Pos1 at iteration " + i + ": " +  pos[1]);
      //Find it the first time
      if (!bFoundBadValue && Math.abs(oldpos[1]-pos[1]) > tolerance){
        badValue = pos[1];
        bFoundBadValue = true;
        System.out.println("Candidate bad value: "+  badValue + " at iteration "+ i );
      }
      //double check and restart - we got unlucky and the first value was the bad one
      else if (bFoundBadValue && Math.abs(oldpos[1]-pos[1]) > tolerance){
        if(Math.abs(badValue- pos[1])< tolerance){
          //found the bad value twice - doesn't work if you get two bads in a row
          System.out.println("Verified bad value: "+  badValue + " at iteration "+ i );
          break;
        }
        else{
          //unluckily found the bad value the first time and now we have it again
          System.out.println("Mistaken bad value: "+  badValue + " at iteration "+ i + " ... Restarting...");
          badValue = oldpos[1];

        }
      }
      oldpos = pos;
      Timer.delay(0.01);
    }
    badTiltValue = badValue;
    return badValue;
  }
  public double getBadTilt(){
    return badTiltValue;
  }
  public void log(){
    counter ++;
    if (Math.floorMod(counter, 10) == 0) {
      //if (Math.abs(pos[1]-Robot.drivetrain.getBadTilt())<0.0001){badTilts++;}
     //else {tilt = pos[1] - tiltOffset;}
     double[] pos = getVector();
     double heading = getHeading();
     SmartDashboard.putNumber("Tilt Errors", badTilts);
     SmartDashboard.putNumber("X", pos[0]);
     SmartDashboard.putNumber("Y", pos[1]);
     SmartDashboard.putNumber("Z", pos[2]);
     SmartDashboard.putNumber("Heading", heading);
    }
  }
}
