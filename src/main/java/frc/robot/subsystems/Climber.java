// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    /** Creates a new Climber. */
    public Climber() {
        // TODO construct and configure the winch motor
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    private boolean canWinch() {
        // TODO
        // Use
        // https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj/Timer.html#getMatchTime()
        /*
         * Timer.getMatchTime()
         */
        // and only release the winch if the match time is
        // past some threshold. That way, we don't accidentally
        // release the winch.
        return false;
    }

    /**
     * releaseClimbWinch releases the climbing winch to allow for climbing
     */
    public Command releaseClimbWinch() {
        Command c = Commands.either(//
                this.runOnce(() -> { // canWinch == TRUE
                    // TODO
                    // Then, apply power to the motor such that the
                    // winch is released.
                    DriverStation.reportWarning("Please implement me!",
                            Thread.currentThread().getStackTrace());
                }).andThen(Commands.waitSeconds(0.5)).andThen(this.stopWinch()), //
                this.runOnce(() -> { // canWinch == FALSE
                    DriverStation.reportWarning(
                            "tried to release climber winch, but cannot do that yet",
                            Thread.currentThread().getStackTrace());
                }), //
                this::canWinch);
        c.addRequirements(this);
        return c;
    }

    /**
     * winchUp has the effect of making the robot "climb" the cage
     */
    public Command winchUp() {
        return this.runOnce(() -> {
            if (!this.canWinch()) {
                DriverStation.reportWarning("tried to 'climb', but cannot do that yet",
                        Thread.currentThread().getStackTrace());
                return;
            }
            // TODO
            // make the robot "climb"
            DriverStation.reportWarning("Please implement me!",
                    Thread.currentThread().getStackTrace());
        });
    }

    /**
     * winchDown has the effect of making the robot "UN-climb" (release from) the cage
     */
    public Command winchDown() {
        return this.runOnce(() -> {
            if (!this.canWinch()) {
                DriverStation.reportWarning("tried to 'un-climb', but cannot do that yet",
                        Thread.currentThread().getStackTrace());
                return;
            }
            // TODO
            // make the robot "un-climb"
            DriverStation.reportWarning("Please implement me!",
                    Thread.currentThread().getStackTrace());
        });
    }

    public Command stopWinch() {
        return this.runOnce(() -> {
            // TODO
            // turn off power from the winch motor
            DriverStation.reportWarning("Please implement me!",
                    Thread.currentThread().getStackTrace());
        });
    }
}
