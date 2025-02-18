// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;
import frc.robot.Constants.Settings;

public class CoralHandler extends SubsystemBase {
    /** Creates a new CoralHandler. */


  
    /* Declare here */
    public CoralHandler() {}

    public Command intakeFromStation() {
        return this.runOnce(() -> {
            // TODO turn on the motor to intake
            DriverStation.reportWarning("Please implement me!", null);
        });
    }

    public Command spitItOut() {
        return this.runOnce(() -> {
            // TODO turn on the motor to outtake
            DriverStation.reportWarning("Please implement me!", null);
        });
    }

    public Command stop() {
        return this.runOnce(() -> {
            // TODO stop the motor
            DriverStation.reportWarning("Please implement me!", null);
        });
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
