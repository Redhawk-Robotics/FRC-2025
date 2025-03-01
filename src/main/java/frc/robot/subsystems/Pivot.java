// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Settings;

public class Pivot extends SubsystemBase {
    /** Creates a new CoralPivot. */

    // Declare Motors here
    private final SparkMax leftMotor = new SparkMax(5, MotorType.kBrushless);
    private final SparkAbsoluteEncoder encoder = leftMotor.getAbsoluteEncoder();
    private final SparkClosedLoopController controller = leftMotor.getClosedLoopController();

    private double setPoint = 0;
    private int slotIndex = -1;
    // slotIndex < 0 means to use .set(this.speed)
    // slotIndex 1 means to use PID for Pivot with this.setPoint
    private double speed = 0;

    public Pivot() {
        this.configureMotors(//
                Settings.Pivot.kP, Settings.Pivot.kI, Settings.Pivot.kD);
    }

    // this is only public so we can tune PID
    public void configureMotors(double kP1, double kI1, double kD1) {
        System.out.printf("Configuring Pivot motor\n\t(kP:%f kI:%f kD:%f)...\n", kP1, kI1, kD1);

        SparkMaxConfig globalConfig = new SparkMaxConfig();
        globalConfig.smartCurrentLimit(60).idleMode(IdleMode.kBrake);

        SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
        leftMotorConfig.apply(globalConfig);
        leftMotorConfig.absoluteEncoder//
                .setSparkMaxDataPortConfig()//
                .inverted(true)//
                .zeroOffset(Settings.Pivot.ZERO_OFFSET)//
                .positionConversionFactor(Settings.Pivot.CONVERSION_FACTOR)// 360deg/rev
                .velocityConversionFactor(Settings.Pivot.CONVERSION_FACTOR);
        leftMotorConfig.closedLoop//
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)//
                .pid(kP1, kI1, kD1, ClosedLoopSlot.kSlot1)//
        ;
        // slot 1 == position control
        leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        System.out.println("Done configuring Pivot motors.");
    }

    public double getPosition() {
        return this.encoder.getPosition();
    }

    public void setReference(double reference) {
        System.out.printf("Setting Pivot reference to %f (current %f)\n", reference,
                this.getPosition());
        this.setPoint = reference;
        this.slotIndex = 1;
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public void useSpeed() {
        this.slotIndex = -1;
    }

    private boolean shouldUsePIDControl() {
        return this.slotIndex > 0 && this.slotIndex < ControlType.values().length;
    }

    public void stopPivot() {
        this.useSpeed();
        this.setSpeed(0);
        // redundant
        this.leftMotor.set(0.0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        if (this.shouldUsePIDControl()) {
            this.controller.setReference(this.setPoint, ControlType.kPosition,
                    ClosedLoopSlot.values()[slotIndex]);
        } else {
            this.leftMotor.set(speed);
        }

        SmartDashboard.putBoolean("Pivot/use PID", this.shouldUsePIDControl());
        SmartDashboard.putNumber("Pivot/PID setPoint", this.setPoint);
        SmartDashboard.putNumber("Pivot/PID slot", this.slotIndex);
        SmartDashboard.putBoolean("Pivot/use speed", !this.shouldUsePIDControl());
        SmartDashboard.putNumber("Pivot/target speed", this.speed);

        SmartDashboard.putNumber("Pivot/Motor/speed", this.leftMotor.get());
        SmartDashboard.putNumber("Pivot/Motor/voltage", this.leftMotor.getBusVoltage());
        SmartDashboard.putNumber("Pivot/Position", this.encoder.getPosition());
        SmartDashboard.putNumber("Pivot/Velocity", this.encoder.getVelocity());

        SmartDashboard.putNumber("Pivot/Motor/Current", this.leftMotor.getOutputCurrent());
    }
}
