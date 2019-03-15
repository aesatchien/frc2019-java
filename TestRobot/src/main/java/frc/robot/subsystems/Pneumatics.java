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
  private DoubleSolenoid frontSolenoid = new DoubleSolenoid(1, 2);
  private DoubleSolenoid backSolenoid = new DoubleSolenoid(3, 4);
  private TalonSRX pneumaticTalon = new TalonSRX(6);
  private VictorSPX pneumaticVictor = new VictorSPX(7);
  private boolean bFrontHigh;
  private boolean bBackHigh;
  private boolean bHighGear = false;
  private boolean bCompressorOn=false;
  private boolean bHatchOpen=false;
  private boolean bClimbingEnabled = false;
  private int counter;
  private double tilt;
  private final double frontTiltLimit = 2.0;
  private final double backTiltLimit = 2.0;

    @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
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
    frontSolenoid.set(DoubleSolenoid.Value.kOff);
    backSolenoid.set(DoubleSolenoid.Value.kOff);
    //shifter.set(DoubleSolenoid.Value.kOff);
  }
  // Raise robot while maintaining balance with the gyro
  public void raiseRobot(){
    tilt = Robot.drivetrain.driveGyro.getAngle();
    if (tilt < - frontTiltLimit ) {
      frontSolenoid.set(DoubleSolenoid.Value.kOff);
      backSolenoid.set(DoubleSolenoid.Value.kForward);
    }
    else if (tilt > backTiltLimit) {
      frontSolenoid.set(DoubleSolenoid.Value.kForward);
      backSolenoid.set(DoubleSolenoid.Value.kOff);
    }
    else {
      frontSolenoid.set(DoubleSolenoid.Value.kForward);
      backSolenoid.set(DoubleSolenoid.Value.kForward);
    }
  }

    // Raise robot while maintaining balance with the gyro
    public void lowerRobot(){
      tilt = Robot.drivetrain.driveGyro.getAngle();
      if (tilt < - frontTiltLimit ) {
        frontSolenoid.set(DoubleSolenoid.Value.kReverse);
        backSolenoid.set(DoubleSolenoid.Value.kOff);
      }
      else if (tilt > backTiltLimit) {
        frontSolenoid.set(DoubleSolenoid.Value.kOff);
        backSolenoid.set(DoubleSolenoid.Value.kReverse);
      }
      else {
        frontSolenoid.set(DoubleSolenoid.Value.kReverse);
        backSolenoid.set(DoubleSolenoid.Value.kReverse);
      }
    }

  // Retract both front and back
  public void retractFrontAndBack(){  
    frontSolenoid.set(DoubleSolenoid.Value.kReverse);
    backSolenoid.set(DoubleSolenoid.Value.kReverse);
  }
  public void retractFront(){  
    frontSolenoid.set(DoubleSolenoid.Value.kReverse);
  }
  public void retractBack(){  
    backSolenoid.set(DoubleSolenoid.Value.kReverse);
  }
  
  public void set_tiltOffset(){
    //tiltOffset= (Robot.drivetrain.getVector())[1];
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
      
      double tilt = Robot.drivetrain.driveGyro.getAngle();
      bFrontHigh = (tilt < -frontTiltLimit);
      bBackHigh = (tilt > backTiltLimit);
      SmartDashboard.putBoolean("Front High", bFrontHigh);
      SmartDashboard.putBoolean("Back High", bBackHigh);
      SmartDashboard.putBoolean("Compressor", bCompressorOn);
      SmartDashboard.putNumber("Tilt", 0.01 * Math.round(100 * tilt));
      SmartDashboard.putBoolean("High Gear", bHighGear);
      SmartDashboard.putBoolean("Climbable", bClimbingEnabled);

    }
  }

}
