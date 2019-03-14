/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class Instant_SetWrist extends InstantCommand {
  /**
   * Add your docs here.
   */
  double setpoint;
  public Instant_SetWrist() {
    super();
    requires(Robot.wrist);
  }
  public Instant_SetWrist(double setpoint) {
    this();
    this.setpoint = setpoint;
  }
  // Called once when the command executes
  @Override
  protected void initialize() {
    //Robot.wrist.setPosition(setpoint);
    Robot.wrist.setWristMagic(setpoint);
  }

}
