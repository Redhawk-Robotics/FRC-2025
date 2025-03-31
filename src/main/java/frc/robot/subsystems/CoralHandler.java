// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;
import frc.robot.Constants.Settings;
import com.revrobotics.spark.SparkMax;

public class CoralHandler extends SubsystemBase {

    private final SparkMax coralIntakeMotor;
    private Timer getTime = new Timer();

    /** Creates a new CoralHandler. */
    public CoralHandler() {
        this.coralIntakeMotor = new SparkMax(Ports.CoralIntake.WHEEL_INTAKE,
                Settings.CoralHandler.CORAL_INTAKE_MOTORTYPE);
    }

    // TODO -- if this isn't enough, here's a "smarter" way
    // 
    // bool wantCoral
    // bool haveCoral
    // 
    // onIntakeButton()
    //   wantCoral = true
    //   setSpeed INTAKE
    // onOuttakeButton()
    //   wantCoral = false
    //   haveCoral = false
    //   setSpeed OUTTAKE
    // periodic()
    //   if (wantCoral && !haveCoral)
    //     haveCoral = getOutputCurrent > THRESHOLD
    //   if (haveCoral)
    //     wantCoral = false
    //     setSpeed LOW
    // 
    // Might need to look at this again and check the Stop condition

    public Command intake() {
        return this.runOnce(() -> {
            coralIntakeMotor.set(1); // TODO TUNE
        });
    }

    public Command contain() {
        return this.runOnce(() -> {
            coralIntakeMotor.set(0.05); // TODO TUNE
        });
    }

    public Command spitItOut() {
        return this.runOnce(() -> {
            coralIntakeMotor.set(-0.7); // TODO TUNE
        });
    }

    public Command stop() {
        return this.runOnce(() -> {
            coralIntakeMotor.set(0);
        });
    }

    //TODO CONFIRM THIS
    //TODO MAKE A DEFAULT COMMAND FOR LEDS THAT TAKES CORAL AS A SUBSYTEM
    // RUN CORAL AND INTERRUPT WITH BOOLEAN 
    // * The lambda turns a primitive value into a supplier, good to keep in mind.... - Sevnen


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Coral Handler/Intake motor current",
                coralIntakeMotor.getOutputCurrent());
        SmartDashboard.putNumber("Coral Handler/Motor Output Speed", coralIntakeMotor.get());
        SmartDashboard.putNumber("Coral Handler/Bus Voltage", coralIntakeMotor.getBusVoltage());
        SmartDashboard.putNumber("Coral Handler/Applied Output ",
                coralIntakeMotor.getAppliedOutput());
        SmartDashboard.putNumber("Coral Handler/Timer Value", getTime.get());
    }
}
