// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Settings;

public class AlgaeRoller extends SubsystemBase {

    private final SparkMax rollerMotor = new SparkMax(Settings.AlgaeFloorIntake.CAN.ID_ROLLER,
            Settings.AlgaeFloorIntake.ALGAE_FLOOR_INTAKE_MOTORTYPE);

    public AlgaeRoller() {

        SparkMaxConfig rollerMotorConfig = new SparkMaxConfig();
        SparkMaxConfig globalConfig = new SparkMaxConfig();

        globalConfig.smartCurrentLimit(40).idleMode(IdleMode.kBrake);

        rollerMotorConfig.apply(globalConfig).inverted(true);

    }

    public void setSpeed(double speed) {
        this.rollerMotor.set(speed);
    }

    public double getSpeed() {
        return this.rollerMotor.get();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // SmartDashboard.putNumber("Spoiler/Roller/Position", this.encoder.getPosition());
        // SmartDashboard.putNumber("Spoiler/Roller/Velocity", this.encoder.getVelocity());
    }
}
