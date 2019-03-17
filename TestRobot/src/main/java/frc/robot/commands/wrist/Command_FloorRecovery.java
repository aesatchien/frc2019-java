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
public class Command_FloorRecovery extends InstantCommand {
  /**
   * Add your docs here.
   */
  int position;
  public Command_FloorRecovery() {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.wrist);
  }
  public Command_FloorRecovery(int pos) {
    this();
    this.position = pos;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.wrist.set_Encoder(position);
    Robot.wrist.setWristMagic(0);;
  }

}
