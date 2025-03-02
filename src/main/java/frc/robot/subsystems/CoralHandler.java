// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;
import frc.robot.Constants.Settings;
import com.revrobotics.spark.SparkMax;

public class CoralHandler extends SubsystemBase {
    /** Creates a new CoralHandler. */
    private final SparkMax coralIntakeMotor;

    private Boolean triggeredByOutake;
    private Boolean triggeredByIntake;

    public CoralHandler() {
        // TODO
        this.coralIntakeMotor = new SparkMax(Ports.CoralIntake.WHEEL_INTAKE, Settings.CoralHandler.CORAL_INTAKE_MOTORTYPE);
    }

    public Command intakeFromStation() {
        return this.runOnce(() -> {
            // TODO turn on the motor to intake
            coralIntakeMotor.set(1);
            DriverStation.reportWarning("I am running!", Thread.currentThread().getStackTrace());
        });
    }

    public Command spitItOut() {
        return this.runOnce(() -> {
            
            // TODO turn on the motor to outtake     
            coralIntakeMotor.set(-.7);
            DriverStation.reportWarning("Please implement me!", Thread.currentThread().getStackTrace());
        });
    }

    public Command stop() {
        return this.runOnce(() -> {
            coralIntakeMotor.set(0);
            DriverStation.reportWarning("Please implement me!", Thread.currentThread().getStackTrace());
        });
    }

    //IT WILL BE ASSUMED THAT A CORAL IS INSIDE UNTIL TRIGGERDBYINTAKE IS FALSE
    public void checkIntakeVoltDrop() {
        // CONDITIONS 
        // CORAL IN
        // CORAL OUT 
        if ( coralIntakeMotor.getBusVoltage() < Settings.CoralHandler.CORAL_INTAKE_VOLTAGE ) {
            triggeredByIntake = true;
            triggeredByOutake = false;
        }
    }

    public void checkOuttakeVoltDrop() {
        if ( coralIntakeMotor.getBusVoltage() < Settings.CoralHandler.CORAL_OUTTAKE_VOLTAGE) {
            triggeredByIntake = false;
            triggeredByOutake = true;
        }
    }

    public Boolean wasIntakeTriggered() {
        return triggeredByIntake;
    }

    public Boolean wasOuttakeTriggereD() {
        return triggeredByOutake;
    }

    @Override
    public void periodic() {
        checkIntakeVoltDrop();
        checkOuttakeVoltDrop();
        SmartDashboard.putNumber("Coral intake motor current",coralIntakeMotor.getOutputCurrent());
        SmartDashboard.putNumber("Pivot/Motor Output Speed", coralIntakeMotor.get());
    }
}
