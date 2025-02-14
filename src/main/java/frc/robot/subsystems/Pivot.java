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

public class Pivot extends SubsystemBase {
    /** Creates a new CoralPivot. */

    // Declare Motors here
    SparkMax leftMotor;
    // SparkMax rightMotor; // no longer using two motors for pivot

    public Pivot() {
        this.leftMotor = new SparkMax(5, MotorType.kBrushless);
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
