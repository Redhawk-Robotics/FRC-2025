// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;
import frc.robot.Constants.Settings;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;



public class CoralHandler extends SubsystemBase {
    /** Creates a new CoralHandler. */
    private final SparkMax coralIntakeMotor;

    public CoralHandler() {
        // TODO
        this.coralIntakeMotor = new SparkMax(Ports.CoralIntake.WHEEL_INTAKE, Settings.Pivot.CORAL_INTAKE_MOTORTYPE);
    }

    public Command intakeFromStation() {
        return this.runOnce(() -> {
            // TODO turn on the motor to intake
            coralIntakeMotor.set(1);
            DriverStation.reportWarning("Please implement me!", Thread.currentThread().getStackTrace());
        });
    }

    public Command spitItOut() {
        return this.runOnce(() -> {
            
            // TODO turn on the motor to outtake
            coralIntakeMotor.set(-1);
            DriverStation.reportWarning("Please implement me!", Thread.currentThread().getStackTrace());
        });
    }

    public Command stop() {
        return this.runOnce(() -> {
            coralIntakeMotor.set(0);
            DriverStation.reportWarning("Please implement me!", Thread.currentThread().getStackTrace());
        });
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Coral intake motor current",coralIntakeMotor.getOutputCurrent());
    }
}
