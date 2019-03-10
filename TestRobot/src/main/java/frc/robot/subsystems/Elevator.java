/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Command_ElevatorDefault;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Elevator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private int counter;
  private final Spark PWMSpark = new Spark(1);
  private final DigitalInput elevatorLimitLow = new DigitalInput(3);
  //AMT encoder - use the brown on B 1 and white on A on 0 ??
  private Encoder elevatorEncoder = new Encoder(0,1,false, Encoder.EncodingType.k4X);
  double distancePerPulse = 2048;
  double maxPower = 0.6;
  double elevatorPower = 0;
  double setpoint = 0;
  
public Elevator(){
  super();
  elevatorEncoder.setDistancePerPulse(distancePerPulse);
  elevatorEncoder.setDistancePerPulse((1.0*3.14)/distancePerPulse);
  //driveEncoder.setReverseDirection(true);
	elevatorEncoder.setSamplesToAverage(7);
	elevatorEncoder.reset();
}


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new Command_ElevatorDefault());
  }

  public void setElevatorPower(double pow) {
    elevatorPower= pow;
    PWMSpark.set(elevatorPower);
  }

  public double getElevatorHeight(){
    return elevatorEncoder.getDistance();
  }
  public double getElevatorSetpoint(){
    return setpoint;
  }
  public void setElevatorSetpoint(double height){
    setpoint = height;
  }
  public boolean isElevatorLow(){
    return !elevatorLimitLow.get();
  }
  public void reset(){
    elevatorEncoder.reset();
    elevatorPower = 0;
    setpoint = 0;
  }
  public void log() {
    counter ++;
    if (Math.floorMod(counter, 10) == 0) {
      SmartDashboard.putBoolean("Elevator Bottom", !elevatorLimitLow.get());
      SmartDashboard.putNumber("Elevator Distance", ((int)(100*elevatorEncoder.getDistance()))/100.0);
      SmartDashboard.putNumber("Elevator Power", elevatorPower);
      SmartDashboard.putNumber("Elevator Setpoint",(int)(100*setpoint)/100.0);
    }
   // if (Math.floorMod(counter, 100) == 0) {
   //   if (isElevatorLow()) {elevatorEncoder.reset();}
   // }
  }
}
