/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.spartanutils;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.Joystick;

/**
 * Add your docs here.
 */
public class AxisButton extends Button {
    // declare variables
    Joystick joystick;
    int axis;
    double threshold = 0.01;
    
    public AxisButton(Joystick joystick, int axis) {
        // assign variables
        this.joystick = joystick;
        this.axis = axis;
    }

    public boolean get() {
        return joystick.getRawAxis(axis) > threshold;
    }
}