// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
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
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Settings;

@Logged
public class Pivot extends SubsystemBase {
    /** Creates a new CoralPivot. */

    // Declare Motors here
    private final SparkMax leftMotor = new SparkMax(5, MotorType.kBrushless);
    private final SparkAbsoluteEncoder encoder = leftMotor.getAbsoluteEncoder();
    private final SparkClosedLoopController controller = leftMotor.getClosedLoopController();

    public enum Mode {
        kManualSpeed, kManualPosition, kPosition;
    }

    Mode mControlMode = Mode.kManualSpeed;
    private DoubleSupplier mInput = () -> 0.;
    double mSetPoint;

    public Pivot() {
        this.configureMotors(//
                Settings.Pivot.kP, Settings.Pivot.kI, Settings.Pivot.kD);
    }

    // this is only public so we can tune PID
    public void configureMotors(double kP1, double kI1, double kD1) {
        System.out.printf("Configuring Pivot motor\n\t(kP:%f kI:%f kD:%f)...\n", kP1, kI1, kD1);

        SparkMaxConfig globalConfig = new SparkMaxConfig();
        globalConfig.smartCurrentLimit(40).idleMode(IdleMode.kBrake).inverted(true);

        SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
        leftMotorConfig.apply(globalConfig);

        // leftMotorConfig.softLimit//
        //         .forwardSoftLimitEnabled(false)//
        //         .forwardSoftLimit(0.05)//
        //         .reverseSoftLimitEnabled(true)//
        //         .reverseSoftLimit(0.5);

        leftMotorConfig.absoluteEncoder//
                .setSparkMaxDataPortConfig()//
                .inverted(false)// TODO -- check this?
                .zeroOffset(Settings.Pivot.ZERO_OFFSET)//
                .positionConversionFactor(Settings.Pivot.CONVERSION_FACTOR)// 360deg/rev
                .velocityConversionFactor(Settings.Pivot.CONVERSION_FACTOR);
        leftMotorConfig.closedLoop//
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)//
                .pid(kP1, kI1, kD1, ClosedLoopSlot.kSlot1)//
                .maxOutput(0.8)//
                .minOutput(-0.8);//
        ;
        // slot 1 == position control
        leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        System.out.println("Done configuring Pivot motors.");
    }

    public double getPosition() {
        return this.encoder.getPosition();
    }

    public void useControlMode(Mode mode, double initialState, DoubleSupplier input) {
        this.mControlMode = mode;
        this.mSetPoint = initialState;
        if (input != null) {
            this.mInput = input;
        }
    }

    public void stopPivot() {
        this.useControlMode(Mode.kManualSpeed, 0, () -> 0.);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot/setpoint-input", this.mInput.getAsDouble());
        switch (this.mControlMode) {
            case kManualSpeed:
                // interpret the input func as directly setting the speed
                this.mSetPoint = MathUtil.clamp(this.mInput.getAsDouble(), -1., 1.);
                this.leftMotor.set(this.mSetPoint);
                break;
            case kManualPosition:
                // interpet the input func as an addition to the setpoint
                this.mSetPoint = MathUtil.clamp(this.mSetPoint + this.mInput.getAsDouble(),
                        Settings.Positioner.minPivotPosition, Settings.Positioner.maxPivotPosition);
                this.controller.setReference(this.mSetPoint, ControlType.kPosition,
                        ClosedLoopSlot.kSlot1);
                break;
            case kPosition:
                // interpet the input func as directly setting the position setpoint
                this.mSetPoint = MathUtil.clamp(this.mInput.getAsDouble(),
                        Settings.Positioner.minPivotPosition, Settings.Positioner.maxPivotPosition);
                this.controller.setReference(this.mSetPoint, ControlType.kPosition,
                        ClosedLoopSlot.kSlot1);
                break;
            default:
                break;
        }
        SmartDashboard.putString("Pivot/Control-Mode", this.mControlMode.toString());
        SmartDashboard.putNumber("Pivot/Set-Point", this.mSetPoint);

        SmartDashboard.putNumber("Pivot/Position", this.encoder.getPosition());
        SmartDashboard.putNumber("Pivot/Velocity", this.encoder.getVelocity());

        SmartDashboard.putNumber("Pivot/Motor/Current", this.leftMotor.getOutputCurrent());
    }
}
