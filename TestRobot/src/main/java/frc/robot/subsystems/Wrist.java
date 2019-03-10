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
  private final double WRIST_POWER_FORWARD_LIMIT = 0.7;
	private final double WRIST_POWER_REVERSE_LIMIT = -0.65;
  
  private int counter = 0;

  public Wrist(){
    super();
    wristEncoder.setDistancePerPulse(distancePerPulse);
    wristEncoder.setDistancePerPulse((1.0/31.0)/distancePerPulse);
    wristEncoder.setReverseDirection(true);
    wristEncoder.setSamplesToAverage(7);
    wristEncoder.reset();
    wristTalon.setInverted(true);

    //Initial config for the Talon 
		wristTalon.setSensorPhase(true);  //Need to double check this
		wristTalon.setSelectedSensorPosition(0, 0, 0);  //Position is zero on on the encoder when we boot up
    //set the speed limits
    wristTalon.configPeakOutputForward(WRIST_POWER_FORWARD_LIMIT, 10);
    wristTalon.configPeakOutputReverse(WRIST_POWER_REVERSE_LIMIT, 10);  //Gravity is our reverse
    //See if the Voltage compensation will give us more reliable performance
    wristTalon.configVoltageCompSaturation(11, 10);
    wristTalon.enableVoltageCompensation(true);
    
		//Set the gains - do this after testing it with the joystick
		//FWD, P, I, D, I limits that work ok for position mode (in Talon SLOT 0)
    // wristTalon.config_kP(0, .25, 10);  // 0.25 works if you don't limit the max setpoint difference
    wristTalon.config_kP(0, .8, 10);  // 0.8 works if you DO limit the max setpoint difference to ~ 500
    wristTalon.config_kI(0, 0, 10);
    wristTalon.config_kD(0, 10, 10);
    wristTalon.config_kF(0, 0, 10);
    
    //FWD, P, I, D, I limits that work ok for velocity mode (in Talon SLOT 1)
    wristTalon.config_kP(1, 2.0, 10);
    wristTalon.config_kI(1, 0.002, 10);
    wristTalon.config_kD(1, 0.0, 10);
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
    @Deprecated
    //This is for controlling via the TalonSRX in velocity mode
    //NEEDS A LOT OF WORK
  public void setVelocity(double vel) {
    //Velocity we put in Slot 1, PID 0
    wristTalon.selectProfileSlot(1, 0);
    if (vel < 0){
      //FWD, P, I, D, F limits that work ok for velocity mode (in Talon SLOT 1)
      wristTalon.config_kP(1, 2.0, 10);
      wristTalon.config_kI(1, 0.002, 10);
      wristTalon.config_kD(1, 0.0, 10);
      wristTalon.config_kF(0, 0, 10);
    }
    else {
    //FWD, P, I, D, F limits that work ok for velocity mode (in Talon SLOT 1)
      wristTalon.config_kP(1, 2.0, 10);
      wristTalon.config_kI(1, 0.002, 10);
      wristTalon.config_kD(1, 0.0, 10);
      wristTalon.config_kF(0, 0, 10);
    }
    wristTalon.set(ControlMode.Velocity, vel ,1);
    wristTalon.configOpenloopRamp(0.1, 10); 
  }


  
  public void log() {
    counter ++;
    if (Math.floorMod(counter, 10) == 0) {
      //SmartDashboard.putBoolean("Wrist Top", !elevatorLimitLow.get());
      SmartDashboard.putNumber("Wrist Distance", ((int)(100*wristEncoder.getDistance()))/100.0);
      double wristPos = wristTalon.getSelectedSensorPosition(0);
      SmartDashboard.putNumber("Wrist Talon", ((int)(100*wristPos))/100.0);
      SmartDashboard.putNumber("Wrist Velocity", wristTalon.getSelectedSensorVelocity(0));
    }
  }
  
}
