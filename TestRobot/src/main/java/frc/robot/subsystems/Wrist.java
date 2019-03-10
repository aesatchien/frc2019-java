/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * Add your docs here.
 */
public class Wrist extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.  
  //AMT encoder - use the brown on B 1 and white on A on 0 ??
  private Encoder wristEncoder = new Encoder(4,5,false, Encoder.EncodingType.k4X);
  double distancePerPulse = 2048;
  private TalonSRX wristTalon = new TalonSRX(5);
  
  
  private int counter = 0;

  public Wrist(){
    super();
    wristEncoder.setDistancePerPulse(distancePerPulse);
    wristEncoder.setDistancePerPulse((1.0/31.0)/distancePerPulse);
    wristEncoder.setReverseDirection(true);
    wristEncoder.setSamplesToAverage(7);
    wristEncoder.reset();
    wristTalon.setInverted(true);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void reset(){
    wristEncoder.reset();
  }

  public void setWristPower(double pow) {
    wristTalon.set(ControlMode.PercentOutput, pow);
  }
  public void moveWrist(double direction) {
    double maxPower = 0.4;
    //move up - pull hardest at the bottom (encoder = 1)
    if (direction > 0){
      wristTalon.set(ControlMode.PercentOutput, Math.max( maxPower ,0.25*wristEncoder.getDistance()));
    }
    //move down - try to get it to balance on the way down by giving a bit of positive
    else if (direction < 0){
      wristTalon.set(ControlMode.PercentOutput, -0.2 + 0.4*wristEncoder.getDistance());
    }
    else{
      wristTalon.set(ControlMode.PercentOutput, 0);
    }
    
  }
  public void log() {
    counter ++;
    if (Math.floorMod(counter, 10) == 0) {
      //SmartDashboard.putBoolean("Wrist Top", !elevatorLimitLow.get());
      SmartDashboard.putNumber("Wrist Distance", ((int)(100*wristEncoder.getDistance()))/100.0);
    }
  }
  
}
