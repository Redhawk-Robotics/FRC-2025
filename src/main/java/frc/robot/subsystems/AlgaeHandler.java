// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Settings;

public class AlgaeHandler extends SubsystemBase {

    private final SparkMax algaeHandlerMotor;

    /** Creates a new AlgaeHandler. */
    public AlgaeHandler() {
        this.algaeHandlerMotor = new SparkMax(Settings.AlgaeHandler.CAN.ID_MOTOR,
                Settings.AlgaeHandler.ALGAE_INTAKE_MOTORTYPE);
        // TODO configure motor
    }

    public Command rotateCW() {
        return this.runOnce(() -> {
            this.setSpeed(1);
        });
    }

    public Command rotateCCW() {
        return this.runOnce(() -> {
            this.setSpeed(-1);
        });
    }

    public Command contain() {
        return this.runOnce(() -> {
            this.setSpeed(0.05);
        });
    }

    public Command stop() {
        return this.runOnce(() -> {
            this.setSpeed(0);
        });
    }

    public void setSpeed(double speed) {
        this.algaeHandlerMotor.set(speed);
    }

    public double getSpeed() {
        return this.algaeHandlerMotor.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Algae Handler/motor current",
                this.algaeHandlerMotor.getOutputCurrent());
    }
}
