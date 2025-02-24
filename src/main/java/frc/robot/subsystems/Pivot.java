// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import java.util.function.Supplier;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {
    /** Creates a new CoralPivot. */

    // Declare Motors here
    private final SparkMax leftMotor;
    private final SparkAbsoluteEncoder encoder;
    private final SparkClosedLoopController controller;

    public Pivot() {
        this.leftMotor = new SparkMax(5, MotorType.kBrushless);

        this.configureMotors(0.1, 0, 0);

        this.encoder = leftMotor.getAbsoluteEncoder();
        this.controller = leftMotor.getClosedLoopController();
    }

    // this is only public so we can tune PID
    public void configureMotors(double kP, double kI, double kD) {
        System.out.printf("Configuring Pivot motors (kP:%f kI:%f kD:%f)...\n", kP, kI, kD);

        SparkMaxConfig globalConfig = new SparkMaxConfig();
        globalConfig.smartCurrentLimit(60).idleMode(IdleMode.kBrake);

        SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
        leftMotorConfig.apply(globalConfig);
        double zeroOffset = 0.8;
        double conversionFactor = 100./0.5;
        // double conversionFactor = Degrees.of(270).minus(Degrees.of(90)).magnitude();
        leftMotorConfig.absoluteEncoder//
                .setSparkMaxDataPortConfig()//
                .inverted(true)//
                .zeroOffset(zeroOffset)//
                .positionConversionFactor(conversionFactor)// 360deg/rev
                .velocityConversionFactor(conversionFactor);
        leftMotorConfig.closedLoop//
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)//
                .pid(kP,kI,kD)//
                ;
                        // .maxMotion//
                        //         .maxVelocity(5)//
                        //         .maxAcceleration(4)//
                        //         .allowedClosedLoopError(3);
        leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        System.out.println("Done configuring Pivot motors.");
    }

    public Command applySpeedRequest(Supplier<Double> speed) {
        if (speed == null) {
            DriverStation.reportWarning(
                    "Pivot applySpeedRequest received null position supplier",
                    Thread.currentThread().getStackTrace());
            return Commands.none();
        }
        // SubsystemBase.runOnce implicitly requires `this` subsystem.
        return this.runOnce(() -> {
            this.leftMotor.set(speed.get());
        });
    }

    public Command applyAddPositionRequest(Supplier<Double> position) {
        if (position == null) {
            DriverStation.reportWarning(
                    "Pivot applyAddPositionRequest received null position supplier",
                    Thread.currentThread().getStackTrace());
            return Commands.none();
        }
        // SubsystemBase.runOnce implicitly requires `this` subsystem.
        return this.runOnce(() -> {
            // this.leftMotor.set(position.get());
            this.setPosition(this.getPosition() + position.get());
        });
    }

    public double getPosition() {
        return this.encoder.getPosition();
    }

    public void setPosition(double position) {
        this.controller.setReference(position, ControlType.kPosition);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Pivot/Motor/speed", this.leftMotor.get());
        SmartDashboard.putNumber("Pivot/Motor/voltage", this.leftMotor.getBusVoltage());
        SmartDashboard.putNumber("Pivot/Position", this.encoder.getPosition());
        SmartDashboard.putNumber("Pivot/Velocity", this.encoder.getVelocity());
    }
}
