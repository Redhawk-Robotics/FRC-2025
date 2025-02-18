// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;
import frc.robot.Constants.Settings;

public class AlgaeHandler extends SubsystemBase {
    /** Creates a new AlgaeHandler. */

    private final SparkMax intakeMotor;
    private final SparkMax rollerMotor;

    private final SparkMaxConfig intakeMotorConfig;
    private final SparkMaxConfig rollerMotorConfig;
    private final SparkMaxConfig globalConfig;

    public AlgaeHandler() {
        intakeMotor = new SparkMax(Ports.AlgaeHandler.kCANID_INTAKE, Settings.AlgaeHandler.INTAKE_MOTORTYPE);
        rollerMotor = new SparkMax(Ports.AlgaeHandler.kCANID_ROLLER, Settings.AlgaeHandler.ROLLER_MOTORTYPE);

        intakeMotorConfig = new SparkMaxConfig();
        rollerMotorConfig = new SparkMaxConfig();
        globalConfig = new SparkMaxConfig();

        globalConfig.smartCurrentLimit(60).idleMode(IdleMode.kBrake);

        intakeMotorConfig.apply(globalConfig).inverted(false);
        rollerMotorConfig.apply(globalConfig).inverted(false);

        // && CONFIGURING BOTH MOTORS
        intakeMotor.configure(
            globalConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        rollerMotor.configure(globalConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    public Command rotateCW() {
        return this.runOnce(() -> {
            // TODO turn the motor CW
            intakeMotor.set(0.8);
            intakeMotor.set(0.8);
            DriverStation.reportWarning("Please implement me!", null);
        });
    }

    public Command rotateCCW() {
        return this.runOnce(() -> {
            // TODO turn the motors CCW
            intakeMotor.set(-0.8);
            rollerMotor.set(-0.8);
            DriverStation.reportWarning("Please implement me!", null);
        });
    }

    public Command stop() {
        return this.runOnce(() -> {
            intakeMotor.set(0.0);
            intakeMotor.set(0.0);
            DriverStation.reportWarning("Please implement me!", null);
        });
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
