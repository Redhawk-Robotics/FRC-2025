// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;
import frc.robot.Constants.Settings;

public class Pivot extends SubsystemBase {

    // Declare Motors here
    private final SparkMax pivotMotorOne;
    private final SparkMaxConfig pivotMotorConfig;

     /** Creates a new CoralPivot. */
    public Pivot() {
        
        pivotMotorOne = new SparkMax(Ports.CoralIntake.PIVOT_RIGHT, Settings.Pivot.RIGHT_PIVOT_MOTORTYPE);
        pivotMotorConfig = new SparkMaxConfig();
    }

    public Command applySpeeds(double speed) {
        // TODO
        return Commands.none();
    }

    public Command L1() {
        return this.runOnce(() -> {
            // TODO this should put the pivot in the L1 position
            DriverStation.reportWarning("Please implement me!", null);
        });
    }

    public Command L2() {
        return this.runOnce(() -> {
            // TODO this should put the pivot in the L2 position
            DriverStation.reportWarning("Please implement me!", null);
        });
    }

    public Command L3() {
        return this.L2(); // L3 == L2 pivot position
    }

    public Command L4() {
        return this.runOnce(() -> {
            // TODO this should put the pivot in the L4 position
            DriverStation.reportWarning("Please implement me!", null);
        });
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
