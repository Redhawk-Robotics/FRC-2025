// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Settings;

public class AlgaeHandler extends SubsystemBase {

    private final TalonFX algaeHandlerMotor = new TalonFX(Settings.AlgaeHandler.CAN.ID_MOTOR);
    private final DutyCycleOut speedControl = new DutyCycleOut(0);

    /** Creates a new AlgaeHandler. */
    public AlgaeHandler() {
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.CurrentLimits.withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(Amps.of(40));

        this.algaeHandlerMotor.getConfigurator().apply(motorConfig);
    }

    public Command rotateCW_Intake() {
        return this.runOnce(() -> {
            this.setSpeed(0.8);
        });
    }

    public Command rotateCCW_Outtake() {
        return this.runOnce(() -> {
            this.setSpeed(-1);
        });
    }

    public Command contain() {
        return this.runOnce(() -> {
            this.setSpeed(0.1);
        });
    }

    public Command stop() {
        return this.runOnce(() -> {
            this.setSpeed(0);
        });
    }

    public void setSpeed(double speed) {
        this.algaeHandlerMotor.setControl(this.speedControl.withOutput(speed));
    }

    public double getSpeed() {
        return this.algaeHandlerMotor.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Algae Handler/motor current",
                this.algaeHandlerMotor.getTorqueCurrent().getValueAsDouble());
    }
}
