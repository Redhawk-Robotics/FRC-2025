// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeFloorIntakeComponents;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.Settings;

public class AlgaeFloorIntakeRoller {

    private final SparkMax rollerMotor = new SparkMax(Settings.AlgaeFloorIntake.CAN.ID_ROLLER,
            Settings.AlgaeFloorIntake.ALGAE_FLOOR_INTAKE_MOTORTYPE);

    public AlgaeFloorIntakeRoller() {

        SparkMaxConfig rollerMotorConfig = new SparkMaxConfig();
        SparkMaxConfig globalConfig = new SparkMaxConfig();

        globalConfig.smartCurrentLimit(60).idleMode(IdleMode.kBrake);

        rollerMotorConfig.apply(globalConfig).inverted(true);

    }

    public void setSpeed(double speed) {
        this.rollerMotor.set(speed / 2.);
    }
}
