// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import com.revrobotics.spark.ClosedLoopSlot;
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
    private final SparkMax leftMotor= new SparkMax(5, MotorType.kBrushless);
    private final SparkAbsoluteEncoder encoder = leftMotor.getAbsoluteEncoder();
    private final SparkClosedLoopController controller = leftMotor.getClosedLoopController();

    private double setPoint;
    private ControlType controlType;
    private int slotIndex;

    public Pivot() {

        this.configureMotors(//
                0.1, 0, 0, //
                0.025, 0, 0);

    }

    // this is only public so we can tune PID
    public void configureMotors(double kP0, double kI0, double kD0, double kP1, double kI1,
            double kD1) {
        System.out.printf("Configuring Pivot motors (kP:%f kI:%f kD:%f)(kP:%f kI:%f kD:%f)...\n",
                kP0, kI0, kD0, kP1, kI1, kD1);

        SparkMaxConfig globalConfig = new SparkMaxConfig();
        globalConfig.smartCurrentLimit(60).idleMode(IdleMode.kBrake);

        SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
        leftMotorConfig.apply(globalConfig);
        double zeroOffset = 0.8;
        double conversionFactor = 100. / 0.5;
        // double conversionFactor = Degrees.of(270).minus(Degrees.of(90)).magnitude();
        leftMotorConfig.absoluteEncoder//
                .setSparkMaxDataPortConfig()//
                .inverted(true)//
                .zeroOffset(zeroOffset)//
                .positionConversionFactor(conversionFactor)// 360deg/rev
                .velocityConversionFactor(conversionFactor);
        leftMotorConfig.closedLoop//
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)//
                .pid(kP0, kI0, kD0, ClosedLoopSlot.kSlot0)//
                .pid(kP1, kI1, kD1, ClosedLoopSlot.kSlot1)//
        ;
        leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        System.out.println("Done configuring Pivot motors.");
    }

    public double getPosition() {
        return this.encoder.getPosition();
    }

    public Command setReferenceRequest(Supplier<Double> reference, ControlType type) {
        if (reference == null) {
            DriverStation.reportError(
                    "Pivot.setReferenceRequest received null reference supplier",
                    Thread.currentThread().getStackTrace());
            return Commands.none();
        }
        return this.runOnce(() -> {
            this.setPoint = reference.get();
            this.controlType = type;
            // if velocity control, use slot 0
            // otherwise use slot 1
            this.slotIndex = type == ControlType.kVelocity ? 0 : 1;
        });
    }

    // just for a quick test, you can delete later
    public void stopPivotMotor() {
        this.leftMotor.set(0.0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        this.controller.setReference(this.setPoint, this.controlType,
                ClosedLoopSlot.values()[slotIndex]);

        SmartDashboard.putNumber("Pivot/set point", this.setPoint);
        SmartDashboard.putString("Pivot/control type", this.controlType.name());
        SmartDashboard.putNumber("Pivot/slot", this.slotIndex);

        SmartDashboard.putNumber("Pivot/Motor/speed", this.leftMotor.get());
        SmartDashboard.putNumber("Pivot/Motor/voltage", this.leftMotor.getBusVoltage());
        SmartDashboard.putNumber("Pivot/Position", this.encoder.getPosition());
        SmartDashboard.putNumber("Pivot/Velocity", this.encoder.getVelocity());
    }
}
