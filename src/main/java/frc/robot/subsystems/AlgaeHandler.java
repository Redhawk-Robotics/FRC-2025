// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;
import frc.robot.Constants.Settings;

public class AlgaeHandler extends SubsystemBase {
    /** Creates a new AlgaeHandler. */
    private final SparkMax algaeHandlerMotor;
  

    public AlgaeHandler() {
        this.algaeHandlerMotor = new SparkMax(Ports.AlgaeHandler.ALGAEINTAKE_MOTOR, Settings.AlgaeHandler.ALGAE_INTAKE_MOTORTYPE);
        // TODO
    }

    public Command rotateCW() {
        return this.runOnce(() -> {
            // TODO turn the motor CW
            algaeHandlerMotor.set(1);
        });
    }

    public Command rotateCCW() {
        return this.runOnce(() -> {
            // TODO turn the motor CCW
            algaeHandlerMotor.set(-1);
        });
    }

    public Command stop() {
        return this.runOnce(() -> {
            // TODO stop the motor
            algaeHandlerMotor.set(0);
        });
    }



    @Override
    public void periodic() {
         SmartDashboard.putNumber("Algae handler motor current",algaeHandlerMotor.getOutputCurrent());

         
    }
}
