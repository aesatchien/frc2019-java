/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import java.lang.Math;
import edu.wpi.first.wpilibj.Timer;


/**
 * Add your docs here.
 */
public class Pneumatics extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private Solenoid shifter = new Solenoid(0);
  private Solenoid hatch = new Solenoid(5);
  private Compressor compressor = new Compressor(0);
  // The syntax is DoubleSolenoid(fwdch, revch);
  private DoubleSolenoid frontRightSolenoid = new DoubleSolenoid(1, 2);
  private DoubleSolenoid frontLeftSolenoid = new DoubleSolenoid(6, 7);
  private DoubleSolenoid backSolenoid = new DoubleSolenoid(3, 4);
  private TalonSRX pneumaticTalon = new TalonSRX(6);
  private VictorSPX pneumaticVictor = new VictorSPX(7);
  private boolean bFrontHigh;
  private boolean bBackHigh;
  private boolean bLeftHigh;
  private boolean bRightHigh;
  private boolean bHighGear = false;
  private boolean bCompressorOn=false;
  private boolean bHatchOpen=false;
  private boolean bClimbingEnabled = false;
  private int counter;
  private double tilt;
  private double pitch;
  private final double frontTiltLimit = 1.5;
  private final double backTiltLimit = 1.5;
  private final double pitchLimit = 1.0;
  private double pitchOffset = 0;
  private double tiltOffset = 0;

    @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  //set these when you enable climbing
  public void setTiltOffset(){
    tiltOffset = Robot.navigation.getRoll();
  }
  public void setPitchOffset(){
    pitchOffset = Robot.navigation.getPitch();;
  }
  // see if climbing is enabled
  public boolean isClimbingEnabled(){
    return bClimbingEnabled;
  }
  public void setClimbingEnabled(){
    bClimbingEnabled = true;
  }
  public void toggleClimbingEnabled(){
    if(bClimbingEnabled){
      bClimbingEnabled=false;
    }
    else{
      bClimbingEnabled=true;  
    }
  }

  // open and close the hatch
  public void hatchToggle(){
    if(bHatchOpen){
      hatch.set(false);
      bHatchOpen=false;
    }
    else{
      hatch.set(true);
      bHatchOpen=true;  
    }
  }

  // Turn off all front and back so we can "float"
  public void solenoidOff(){
    shifter.set(false);
    frontRightSolenoid.set(DoubleSolenoid.Value.kOff);
    frontLeftSolenoid.set(DoubleSolenoid.Value.kOff);
    backSolenoid.set(DoubleSolenoid.Value.kOff);
    //shifter.set(DoubleSolenoid.Value.kOff);
  }

  // Raise robot while maintaining balance with the gyro
  public void raiseRobot(){
  
   //Try to figure out a way to never let the solenoids be on all the time...
    //double dutycycle = 0.5;
    //if(Timer.getFPGATimestamp()%1.0 > dutycycle){
    //  solenoidOff();
    //  return;
    //}

    tilt = Robot.navigation.getRoll() - tiltOffset;
    pitch = Robot.navigation.getPitch() - pitchOffset;
    bFrontHigh = (tilt < -frontTiltLimit);
    bBackHigh = (tilt > backTiltLimit);
    bLeftHigh = (pitch < - pitchLimit);
    bRightHigh = (pitch > pitchLimit);
    //Let's only write the boolean logic once and rely on the truth table
    int state = 8*(bFrontHigh?1:0) + 4*(bBackHigh?1:0) + 2*(bLeftHigh?1:0) + (bRightHigh?1:0);
    /*  9 conditions - no easy way to do this
    //Robot tilted with no pitch - 2
    //Robot pitched with no tilt  - 2
    //Robot pitched and tilted - 4 
    //Robot neither pitched nor tilted - 1 
    FBLR
    0000 - flat - can also be my else condition
    0001 - R only
    0010 - L only
    0011 - NA
    0100 - B only
    0101 - B and R
    0110 - B and L
    0111 - NA
    1000 - F only 
    1001 - F and R
    1010 - F and L
    1011 - NA
    1100 - NA
    1101 - NA
    1110 - NA
    1111 - NA

    */
    // switch statement 
  switch(state) {
    // case statements
    // values must be of same type of expression
    case 1 :  //R only
      frontRightSolenoid.set(DoubleSolenoid.Value.kOff);
      frontLeftSolenoid.set(DoubleSolenoid.Value.kForward);
      backSolenoid.set(DoubleSolenoid.Value.kOff);
      break; 
    
    case 2 : //L only
      frontRightSolenoid.set(DoubleSolenoid.Value.kForward);
      frontLeftSolenoid.set(DoubleSolenoid.Value.kOff);
      backSolenoid.set(DoubleSolenoid.Value.kOff);
      break; 

    case 4 : //B only
      frontRightSolenoid.set(DoubleSolenoid.Value.kForward);
      frontLeftSolenoid.set(DoubleSolenoid.Value.kForward);
      backSolenoid.set(DoubleSolenoid.Value.kOff);
      break; 

    case 5 : //B and R
      frontRightSolenoid.set(DoubleSolenoid.Value.kOff);
      frontLeftSolenoid.set(DoubleSolenoid.Value.kForward);
      backSolenoid.set(DoubleSolenoid.Value.kOff);
      break; 

    case 6 : //B and L
      frontRightSolenoid.set(DoubleSolenoid.Value.kForward);
      frontLeftSolenoid.set(DoubleSolenoid.Value.kOff);
      backSolenoid.set(DoubleSolenoid.Value.kOff);
      break; 

    case 8 : //F only
      frontRightSolenoid.set(DoubleSolenoid.Value.kOff);
      frontLeftSolenoid.set(DoubleSolenoid.Value.kOff);
      backSolenoid.set(DoubleSolenoid.Value.kForward);
      break; 

    case 9 : //F and R
      frontRightSolenoid.set(DoubleSolenoid.Value.kOff);
      frontLeftSolenoid.set(DoubleSolenoid.Value.kForward);
      backSolenoid.set(DoubleSolenoid.Value.kForward);
      break; 

    case 10: //F and L
      frontRightSolenoid.set(DoubleSolenoid.Value.kForward);
      frontLeftSolenoid.set(DoubleSolenoid.Value.kOff);
      backSolenoid.set(DoubleSolenoid.Value.kForward);
      break; 

    default : 
      frontRightSolenoid.set(DoubleSolenoid.Value.kForward);
      frontLeftSolenoid.set(DoubleSolenoid.Value.kForward);
      backSolenoid.set(DoubleSolenoid.Value.kForward);
  }
}

 // Lower robot while maintaining balance with the gyro - use same code structure as raise robot
  public void lowerRobot(){
    tilt = Robot.navigation.getRoll();
    pitch = Robot.navigation.getPitch();
    bFrontHigh = (tilt < -frontTiltLimit);
    bBackHigh = (tilt > backTiltLimit);
    bLeftHigh = (pitch < - pitchLimit);
    bRightHigh = (pitch > pitchLimit);
    //Let's only write the boolean logic once and rely on the truth table
    int state = 8*(bFrontHigh?1:0) + 4*(bBackHigh?1:0) + 2*(bLeftHigh?1:0) + (bRightHigh?1:0);
    // switch statement 
  switch(state) {
    // case statements
    // values must be of same type of expression
    case 1 :  //R only
      frontRightSolenoid.set(DoubleSolenoid.Value.kReverse);
      frontLeftSolenoid.set(DoubleSolenoid.Value.kOff);
      backSolenoid.set(DoubleSolenoid.Value.kOff);
      break; 
    
    case 2 : //L only
      frontRightSolenoid.set(DoubleSolenoid.Value.kOff);
      frontLeftSolenoid.set(DoubleSolenoid.Value.kReverse);
      backSolenoid.set(DoubleSolenoid.Value.kOff);
      break; 

    case 4 : //B only
      frontRightSolenoid.set(DoubleSolenoid.Value.kOff);
      frontLeftSolenoid.set(DoubleSolenoid.Value.kOff);
      backSolenoid.set(DoubleSolenoid.Value.kReverse);
      break; 

    case 5 : //B and R
      frontRightSolenoid.set(DoubleSolenoid.Value.kReverse);
      frontLeftSolenoid.set(DoubleSolenoid.Value.kOff);
      backSolenoid.set(DoubleSolenoid.Value.kOff);
      break; 

    case 6 : //B and L
      frontRightSolenoid.set(DoubleSolenoid.Value.kOff);
      frontLeftSolenoid.set(DoubleSolenoid.Value.kReverse);
      backSolenoid.set(DoubleSolenoid.Value.kOff);
      break; 

    case 8 : //F only
      frontRightSolenoid.set(DoubleSolenoid.Value.kReverse);
      frontLeftSolenoid.set(DoubleSolenoid.Value.kReverse);
      backSolenoid.set(DoubleSolenoid.Value.kOff);
      break; 

    case 9 : //F and R
      frontRightSolenoid.set(DoubleSolenoid.Value.kReverse);
      frontLeftSolenoid.set(DoubleSolenoid.Value.kOff);
      backSolenoid.set(DoubleSolenoid.Value.kOff);
      break; 

    case 10: //F and L
      frontRightSolenoid.set(DoubleSolenoid.Value.kOff);
      frontLeftSolenoid.set(DoubleSolenoid.Value.kReverse);
      backSolenoid.set(DoubleSolenoid.Value.kOff);
      break; 

    default : 
      frontRightSolenoid.set(DoubleSolenoid.Value.kReverse);
      frontLeftSolenoid.set(DoubleSolenoid.Value.kReverse);
      backSolenoid.set(DoubleSolenoid.Value.kReverse);
  }
}

/*
    // Lower robot while maintaining balance with the gyro
    public void lowerRobot(){
      tilt = Robot.drivetrain.driveGyro.getAngle();
      if (tilt < - frontTiltLimit ) {
        frontLeftSolenoid.set(DoubleSolenoid.Value.kReverse);
        frontRightSolenoid.set(DoubleSolenoid.Value.kReverse);
        backSolenoid.set(DoubleSolenoid.Value.kOff);
      }
      else if (tilt > backTiltLimit) {
        frontLeftSolenoid.set(DoubleSolenoid.Value.kOff);
        frontRightSolenoid.set(DoubleSolenoid.Value.kOff);
        backSolenoid.set(DoubleSolenoid.Value.kReverse);
      }
      else {
        frontLeftSolenoid.set(DoubleSolenoid.Value.kReverse);
        frontRightSolenoid.set(DoubleSolenoid.Value.kReverse);
        backSolenoid.set(DoubleSolenoid.Value.kReverse);
      }
    }
*/
  // Retract both front and back - probably should put this on the gyro as well... copy the code from above
  public void retractFrontAndBack(){  
    frontLeftSolenoid.set(DoubleSolenoid.Value.kReverse);
    frontRightSolenoid.set(DoubleSolenoid.Value.kReverse);
    backSolenoid.set(DoubleSolenoid.Value.kReverse); 
  }
  public void retractFront(){  
    frontLeftSolenoid.set(DoubleSolenoid.Value.kReverse);
    frontRightSolenoid.set(DoubleSolenoid.Value.kReverse);
  }
  public void retractBack(){  
    backSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void highGear(){
    shifter.set(true);
  }
  public void lowGear(){
    shifter.set(false);
  }
  public void gearToggle(){
    if(bHighGear){
      lowGear();
      bHighGear=false;
    }
    else{
      highGear();
      bHighGear=true;  
    }
  }
  public void pneumaticDrive(double speed){
    pneumaticTalon.set(ControlMode.PercentOutput, speed);
    pneumaticVictor.set(ControlMode.PercentOutput, -1.0*speed);
  }
  //set the compressor off and on - mainly for practice
  public void compressorOff(){
    compressor.setClosedLoopControl(false);
    bCompressorOn=false;
  }
  public void compressorOn(){
    compressor.setClosedLoopControl(true);
    bCompressorOn=true;
  }
  public void compressorToggle(){
    if(bCompressorOn){
      compressor.setClosedLoopControl(false);
      bCompressorOn=false;
    }
    else{
      compressor.setClosedLoopControl(true);
      bCompressorOn=true;  
    }
  }

  public void reset(){
    bClimbingEnabled= false;
    shifter.set(false);
  }

  public void log(){
    counter ++;
    if (Math.floorMod(counter, 10) == 0) {
      //double tilt = Robot.drivetrain.driveGyro.getAngle();
      tilt = Robot.navigation.getRoll()-tiltOffset;
      pitch = Robot.navigation.getPitch() - pitchOffset;
      bFrontHigh = (tilt < -frontTiltLimit);
      bBackHigh = (tilt > backTiltLimit);
      bLeftHigh = (pitch < - pitchLimit);
      bRightHigh = (pitch > pitchLimit);
      SmartDashboard.putBoolean("Front High", bFrontHigh);
      SmartDashboard.putBoolean("Back High", bBackHigh);
      SmartDashboard.putBoolean("Left High", bLeftHigh);
      SmartDashboard.putBoolean("Right High", bRightHigh);
      SmartDashboard.putBoolean("Compressor", bCompressorOn);
      //SmartDashboard.putNumber("Tilt", 0.01 * Math.round(100 * tilt));
      SmartDashboard.putBoolean("High Gear", bHighGear);
      SmartDashboard.putBoolean("Climbable", bClimbingEnabled);

    }
  }

}
