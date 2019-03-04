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


/**
 * Add your docs here.
 */
public class Pneumatics extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private Solenoid shifter = new Solenoid(0);
  private Compressor compressor = new Compressor(0);
  // The syntax is DoubleSolenoid(fwdch, revch);
  private DoubleSolenoid frontSolenoid = new DoubleSolenoid(1, 2);
  private DoubleSolenoid backSolenoid = new DoubleSolenoid(3, 4);
  private TalonSRX pneumaticTalon = new TalonSRX(6);
  private VictorSPX pneumaticVictor = new VictorSPX(7);
  private boolean bFrontHigh;
  private boolean bBackHigh;
  private int counter;
  private double tilt;
  private double pitch;
  private double yaw;
  private final double frontTiltLimit = 3.0;
  private final double backTiltLimit = 3.0;
  private double tiltOffset = -1.4;
  
    @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public void compressorOff(){
    compressor.setClosedLoopControl(false);
    //compressor.stop();
  }
  public void compressorOn(){
    compressor.setClosedLoopControl(true);
    //compressor.start();
  }
  public void solenoidOff(){
    shifter.set(false);
    frontSolenoid.set(DoubleSolenoid.Value.kOff);
    backSolenoid.set(DoubleSolenoid.Value.kOff);
    //shifter.set(DoubleSolenoid.Value.kOff);
  }
  public void raiseRobot(){
    //tilt = (Robot.drivetrain.getVector())[1];
    if (bFrontHigh) {
      frontSolenoid.set(DoubleSolenoid.Value.kOff);
      backSolenoid.set(DoubleSolenoid.Value.kForward);
    }
    else if (bBackHigh) {
      frontSolenoid.set(DoubleSolenoid.Value.kForward);
      backSolenoid.set(DoubleSolenoid.Value.kOff);
    }
    else {
      frontSolenoid.set(DoubleSolenoid.Value.kForward);
      backSolenoid.set(DoubleSolenoid.Value.kForward);
    }
  }
  public void solenoidReverse(){  
    frontSolenoid.set(DoubleSolenoid.Value.kReverse);
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
  public void pneumaticDrive(double speed){
    pneumaticTalon.set(ControlMode.PercentOutput, speed);
    pneumaticVictor.set(ControlMode.PercentOutput, -1.0*speed);
  }
  public void log(){
    //SmartDashboard.putBoolean("Talon Limit", pneumaticTalon.)
    counter ++;
    
    if (Math.floorMod(counter, 10) == 0) {
      tilt = (Robot.drivetrain.getVector())[1] - tiltOffset;
      bFrontHigh = (tilt > frontTiltLimit);
      bBackHigh = (tilt < -backTiltLimit);
      SmartDashboard.putBoolean("Front High", bFrontHigh);
      SmartDashboard.putBoolean("Back High", bBackHigh);
      SmartDashboard.putNumber("Tilt", tilt);
    }
  }

}
