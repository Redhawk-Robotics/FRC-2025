// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeFloorIntakeComponents;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;
import frc.robot.Constants.Settings;

public class AlgaeFloorIntakeRoller { // TODO do not extend SubsystemBase
    /** Creates a new AlgaeFloorIntakeRoller. */

    private final SparkMax rollerMotor = new SparkMax(Ports.AlgaeFloorIntake.kCAN_ID_ROLLER,
            Settings.AlgaeFloorIntake.ALGAE_FLOOR_INTAKE_MOTORTYPE);

    public AlgaeFloorIntakeRoller() {

        SparkMaxConfig rollerMotorConfig = new SparkMaxConfig();
        SparkMaxConfig globalConfig = new SparkMaxConfig();

        globalConfig.smartCurrentLimit(60).idleMode(IdleMode.kBrake);

        rollerMotorConfig.apply(globalConfig).inverted(true);

    }

    public void setSpeed(double speed) {
        this.rollerMotor.set(speed/2.);
    }



}
