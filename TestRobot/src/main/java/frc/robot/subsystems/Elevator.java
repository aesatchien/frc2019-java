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
  boolean bRocket1, bRocket2, bRocket3, bCargo;
  final double ROCKET1 = 27;
  final double ROCKET2 = 45; 
  final double ROCKET3 = 73; 
  final double CARGO = 48;
  final double MINHEIGHT = 6;
  final double TOLERANCE = 6;

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
  private void unsetFlags(){
    bRocket1= bRocket2= bRocket3= bCargo =false;
  }

  public void reset(){
    elevatorEncoder.reset();
    elevatorPower = 0;
    setpoint = 0;
  }
  public void log() {
    counter ++;
    double encoder = elevatorEncoder.getDistance();
    if (Math.floorMod(counter, 10) == 0) {
      SmartDashboard.putBoolean("Elevator Bottom", !elevatorLimitLow.get());
      SmartDashboard.putNumber("Elevator Distance", ((int)(100*encoder))/100.0);
      SmartDashboard.putNumber("Elevator Power", (int)(100*elevatorPower)/100.0);
      SmartDashboard.putNumber("Elevator Setpoint",(int)(100*setpoint)/100.0);

      unsetFlags();
      double height = encoder + MINHEIGHT;
      if (Math.abs(height-ROCKET1) < TOLERANCE) {bRocket1=true;}
      if (Math.abs(height-ROCKET2) < TOLERANCE) {bRocket2=true;}
      if (Math.abs(height-CARGO) < TOLERANCE) {bCargo=true;}
      if (Math.abs(height-ROCKET3) < TOLERANCE) {bRocket3=true;}
      SmartDashboard.putBoolean("El Rocket1", bRocket1);
      SmartDashboard.putBoolean("El Rocket2", bRocket2);
      SmartDashboard.putBoolean("El Cargo", bCargo);
      SmartDashboard.putBoolean("El Rocket3", bRocket3);

    }
  }
   // if (Math.floorMod(counter, 100) == 0) {
   //   if (isElevatorLow()) {elevatorEncoder.reset();}
   // }
  }
}
