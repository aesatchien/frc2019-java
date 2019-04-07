/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANPIDController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.drivetrain.*;
import java.lang.Math;
import com.revrobotics.ControlType;

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
  //private final SpeedControllerGroup speedGroupLeft = new SpeedControllerGroup(sparkNeoL1, sparkNeoL2);
  //private final SpeedControllerGroup speedGroupRight = new SpeedControllerGroup(sparkNeoR3, sparkNeoR4);
  private final SpeedControllerGroup speedGroupLeft = new SpeedControllerGroup(sparkNeoL1);
  private final SpeedControllerGroup speedGroupRight = new SpeedControllerGroup(sparkNeoR3);
  private final DifferentialDrive  differentialDrive = new DifferentialDrive(speedGroupLeft, speedGroupRight);
  private final CANEncoder SparkNeoEncoder1 = sparkNeoL1.getEncoder();
  private final CANEncoder SparkNeoEncoder3 = sparkNeoR3.getEncoder();
  private CANPIDController sparkPIDControllerRight;
  private CANPIDController sparkPIDControllerLeft;
  //public final Gyro driveGyro = new ADXRS450_Gyro();
  private final double twistSensitivity = 0.99;
  private int counter;
	// Used to make robot accelerate smoother - grinding it
	private static double currentThrust = 0, currentTwist = 0;
	private static double accelerationLimit = 0.1; // was 0.2, seems kinda high
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput,kP2,kI2,kD2,kIz2,kFF2;   


  public Drivetrain() {
    super();
    int currentLimit=80;
    sparkNeoL1.restoreFactoryDefaults();
    sparkNeoL2.restoreFactoryDefaults();
    sparkNeoR3.restoreFactoryDefaults();
    sparkNeoR4.restoreFactoryDefaults();
    sparkNeoL1.setIdleMode(IdleMode.kCoast);
    sparkNeoL2.setIdleMode(IdleMode.kCoast);
    sparkNeoR3.setIdleMode(IdleMode.kCoast);
    sparkNeoR4.setIdleMode(IdleMode.kCoast);
    sparkNeoL1.setSmartCurrentLimit(currentLimit);
    sparkNeoL2.setSmartCurrentLimit(currentLimit);
    sparkNeoR3.setSmartCurrentLimit(currentLimit);
    sparkNeoR4.setSmartCurrentLimit(currentLimit);
    sparkNeoL2.follow(sparkNeoL1);
    sparkNeoR4.follow(sparkNeoR3);
    sparkPIDControllerLeft=sparkNeoL1.getPIDController();
    sparkPIDControllerRight=sparkNeoR3.getPIDController();
    SparkNeoEncoder1.setPositionConversionFactor(4.0*3.141/12.255);
    SparkNeoEncoder3.setPositionConversionFactor(4.0*3.141/12.255);
    // PID coefficients
    kP = 0.06; 
    kI = 0;  //1e-4;
    kD = 0; //3; 
    kIz = 0; 
    kFF = 0.0; 
    kMaxOutput = 0.99; 
    kMinOutput = -0.99;
    sparkPIDControllerLeft.setP(kP);
    sparkPIDControllerLeft.setI(kI);
    sparkPIDControllerLeft.setD(kD);
    sparkPIDControllerLeft.setIZone(kIz);
    sparkPIDControllerLeft.setFF(kFF);
    sparkPIDControllerLeft.setOutputRange(kMinOutput, kMaxOutput);
    sparkPIDControllerRight.setP(kP);
    sparkPIDControllerRight.setI(kI);
    sparkPIDControllerRight.setD(kD);
    sparkPIDControllerRight.setIZone(kIz);
    sparkPIDControllerRight.setFF(kFF);
    sparkPIDControllerRight.setOutputRange(kMinOutput, kMaxOutput);
    kP = 5e-4; 
    kI = 1e-6;  //1e-4;
    kD = 0; //3; 
    kIz = 0; 
    kFF = 0.0; 
    kMaxOutput = 0.99; 
    kMinOutput = -0.99;
    sparkPIDControllerLeft.setP(kP,1);
    sparkPIDControllerLeft.setI(kI,1);
    sparkPIDControllerLeft.setD(kD,1);
    sparkPIDControllerLeft.setIZone(kIz,1);
    sparkPIDControllerLeft.setFF(kFF,1);
    sparkPIDControllerLeft.setOutputRange(kMinOutput, kMaxOutput,1);
    sparkPIDControllerRight.setP(kP,1);
    sparkPIDControllerRight.setI(kI,1);
    sparkPIDControllerRight.setD(kD,1);
    sparkPIDControllerRight.setIZone(kIz,1);
    sparkPIDControllerRight.setFF(kFF,1);
    sparkPIDControllerRight.setOutputRange(kMinOutput, kMaxOutput,1);

   }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new DriveByJoystick());
    //setDefaultCommand(new TankDrive());
  }
  public void SparkWithStick(double xSpeed, double zRotation) { 	
    differentialDrive.arcadeDrive(xSpeed, twistSensitivity*zRotation, false);
    //Do we need to do different things when we are fast/slow?
  }

  public void smoothDrive(double thrust, double twist) {
		//Xbox joysticks are notoriously bad at keeping themselves calibrated to better than 0.1.  
		double deadzone = 0.02;
		if (Math.abs(thrust) < deadzone)
			currentThrust = 0;
		else {
			if (Math.abs(thrust - currentThrust) < accelerationLimit)
				currentThrust = thrust;
			else
				currentThrust = (thrust - currentThrust > 0) ? currentThrust + accelerationLimit : currentThrust - accelerationLimit;
		}

		if (Math.abs(twist) < deadzone)
			currentTwist = 0;
		else {
			if (Math.abs(twist - currentTwist) < accelerationLimit)
				currentTwist = twist;
			else
				currentTwist = (twist - currentTwist > 0) ? currentTwist + accelerationLimit
						: currentTwist - accelerationLimit;
		}
    //Implement a speed limit if necessary
		//currentThrust = Math.signum(currentThrust)*Math.min(Math.abs(thrustLimit),Math.abs(currentThrust));
    differentialDrive.arcadeDrive(currentThrust, currentTwist, true);
  }
  
  //max has to try tank drive ... in the middle of the competition
  public void tankDrive(double left, double right){
    differentialDrive.tankDrive(left,  right); 
  }

  public double getPosition(){
    return SparkNeoEncoder1.getPosition();
  }

  public void setVelocity(double velocity){
    sparkPIDControllerLeft.setReference(-velocity, ControlType.kVelocity,1);
    sparkPIDControllerRight.setReference(velocity, ControlType.kVelocity,1);
    differentialDrive.feed();
  }  

  public void goToSetPoint(double setPoint){
    reset();
    sparkPIDControllerRight.setReference(-setPoint, ControlType.kPosition);
    sparkPIDControllerLeft.setReference(setPoint, ControlType.kPosition);
  }

  public void reset(){
    //driveGyro.reset();
    SparkNeoEncoder1.setPosition(0);
    SparkNeoEncoder3.setPosition(0);

  }

  
  public void log() {
    counter ++;
    if (Math.floorMod(counter, 10) == 0) {
      SmartDashboard.putNumber("Position Enc1", SparkNeoEncoder1.getPosition());
      SmartDashboard.putNumber("Velocity Enc1", SparkNeoEncoder1.getVelocity());
      //SmartDashboard.putNumber("Position Enc2", 0.1*Math.round(10*SparkNeoEncoder2.getPosition()));
      SmartDashboard.putNumber("Position Enc3", SparkNeoEncoder3.getPosition());
      SmartDashboard.putNumber("Velocity Enc3", SparkNeoEncoder3.getVelocity());
      //SmartDashboard.putNumber("Position Enc4", Math.round(SparkNeoEncoder4.getPosition()));
      //SmartDashboard.putNumber("DriveGyro", 0.01*Math.round(100*driveGyro.getAngle()));
      //SmartDashboard.putNumber("BadTiltValue", getBadTilt());
    }
  }
}
