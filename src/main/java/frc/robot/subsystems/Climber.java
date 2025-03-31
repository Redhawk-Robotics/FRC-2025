// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;

public class Climber extends SubsystemBase {
    /** Creates a new Climber. */

    private final SparkMax climberMotor;

    public Climber() {
        this.climberMotor = new SparkMax(Ports.Climber.kCAN_ID_CLIMBER, MotorType.kBrushless);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public Command commandSetClimbSpeed(double speed) {
        return this.runOnce(() -> {
            this.climberMotor.set(speed);
        });
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
        return Commands.none();
    }

    /**
     * winchUp has the effect of making the robot "climb" the cage
     */
    public Command winchUp() {
        return Commands.none();
    }

    /**
     * winchDown has the effect of making the robot "UN-climb" (release from) the cage
     */
    public Command winchDown() {
        return Commands.none();
    }

    public Command stopWinch() {
        return Commands.none();
    }
}
