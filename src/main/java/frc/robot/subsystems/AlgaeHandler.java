// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeHandler extends SubsystemBase {
    /** Creates a new AlgaeHandler. */
    public AlgaeHandler() {
        // TODO
    }

    public Command rotateCW() {
        return this.runOnce(() -> {
            // TODO turn the motor CW
        });
    }

    public Command rotateCCW() {
        return this.runOnce(() -> {
            // TODO turn the motor CCW
        });
    }

    public Command stop() {
        return this.runOnce(() -> {
            // TODO stop the motor
        });
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
