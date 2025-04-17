// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Settings;

// An elevator created with three Krakens instead of 4 Neo/Spark.

// https://v6.docs.ctr-electronics.com/en/latest/docs/migration/migration-guide/control-requests-guide.html

@Logged
public class Elevator extends SubsystemBase {
    /** Creates a new krakenElevator. */
    private final TalonFX m_topRightElevatorMotor = new TalonFX(Settings.Elevator.CAN.ID_TOP_RIGHT); // leader
    private final TalonFX m_topLeftElevatorMotor = new TalonFX(Settings.Elevator.CAN.ID_TOP_LEFT);
    // private final TalonFX m_bottomLeftElevatorMotor =
    //         new TalonFX(Settings.Elevator.CAN.ID_BOTTOM_LEFT);

    // private final PositionTorqueCurrentFOC positionControl = new PositionTorqueCurrentFOC(0);
    private final PositionVoltage positionControl = new PositionVoltage(0);
    private final DutyCycleOut speedControl = new DutyCycleOut(0);

    public enum Mode {
        kManualSpeed, kManualPosition, kPosition;
    }

    Mode mControlMode = Mode.kManualSpeed;
    private DoubleSupplier mInput = () -> 0.;
    double mSetPoint;

    public Elevator() {
        // RobotBase.isReal()
        this.configureMotors(//
                Settings.Elevator.kP, Settings.Elevator.kI, Settings.Elevator.kD);
        this.resetElevatorPosition();
    }

    // Parameters take in values to be configured
    public void configureMotors(double kP1, double kI1, double kD1) {

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.CurrentLimits.withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(Amps.of(60));

        //Volts need to be confirmed
        // motorConfig.Voltage.withPeakForwardVoltage(0).withPeakReverseVoltage(0);

        //Amps need to be confirmed
        // motorConfig.TorqueCurrent.withPeakForwardTorqueCurrent(0)
        //         .withPeakReverseTorqueCurrent(0);

        this.m_topRightElevatorMotor.getConfigurator().apply(motorConfig);
        this.m_topLeftElevatorMotor.getConfigurator().apply(motorConfig);
        // this.m_bottomLeftElevatorMotor.getConfigurator().apply(motorConfig);

        // this.m_topRightElevatorMotor.getConfigurator().apply(
        //         (new TalonFXConfiguration().Slot0.withGravityType(GravityTypeValue.Elevator_Static))
        //                 .withKG(Settings.Elevator.kG)
        //                 .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
        //                 .withKS(Settings.Elevator.kS).withKP(kP1).withKI(kI1).withKD(kD1));

        this.m_topRightElevatorMotor.getConfigurator().apply(
                (new TalonFXConfiguration().Slot0.withGravityType(GravityTypeValue.Elevator_Static))
                        .withKG(0)
                        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                        .withKS(0).withKP(0.75).withKI(0).withKD(0));
        this.m_topRightElevatorMotor.getConfigurator().apply(
                (new TalonFXConfiguration().Slot1.withGravityType(GravityTypeValue.Elevator_Static))
                        .withKG(0)
                        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                        .withKS(0).withKP(0.35).withKI(0).withKD(0));

        this.m_topLeftElevatorMotor
                .setControl(new Follower(Settings.Elevator.CAN.ID_TOP_RIGHT, true));
        // this.m_bottomLeftElevatorMotor
        //         .setControl(new Follower(Settings.Elevator.CAN.ID_TOP_RIGHT, true));

        System.out.println("Done configuring Kraken Elevator motors.");
    }


    public double getPosition() {
        return this.m_topRightElevatorMotor.getPosition().getValueAsDouble();
    }

    public void useControlMode(Mode mode, double initialState, DoubleSupplier input) {
        this.mControlMode = mode;
        this.mSetPoint = initialState;
        if (input != null) {
            this.mInput = input;
        }
    }

    public void stopElevator() {
        this.useControlMode(Mode.kManualSpeed, 0, () -> 0.);
    }

    public void resetElevatorPosition() {
        this.m_topRightElevatorMotor.setPosition(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator/setpoint-input", this.mInput.getAsDouble());
        int slot = 0;
        switch (this.mControlMode) {
            case kManualSpeed:
                // interpret the input func as directly setting the speed
                this.mSetPoint = MathUtil.clamp(this.mInput.getAsDouble(), -1., 1.);
                this.m_topRightElevatorMotor
                        .setControl(this.speedControl.withOutput(this.mSetPoint));
                break;
            case kManualPosition:
                // interpet the input func as an addition to the setpoint
                this.mSetPoint = MathUtil.clamp(this.mSetPoint + this.mInput.getAsDouble(),
                        Settings.Positioner.minElevatorPosition,
                        Settings.Positioner.maxElevatorPosition);
                if (this.mSetPoint < this.getPosition()) {
                    slot = 1;
                }
                this.m_topRightElevatorMotor.setControl(
                        this.positionControl.withPosition(this.mSetPoint).withSlot(slot));
                break;
            case kPosition:
                // interpet the input func as directly setting the position setpoint
                this.mSetPoint = MathUtil.clamp(this.mInput.getAsDouble(),
                        Settings.Positioner.minElevatorPosition, Settings.Positioner.maxElevatorPosition);
                if (this.mSetPoint < this.getPosition()) {
                    slot = 1;
                }
                this.m_topRightElevatorMotor.setControl(
                        this.positionControl.withPosition(this.mSetPoint).withSlot(slot));
                break;
            default:
                break;
        }

        // SmartDashboard.putBoolean("KrakenElevator/use PID", this.shouldUsePIDControl());
        // SmartDashboard.putNumber("KrakenElevator/PID setPoint", this.setPoint);
        // SmartDashboard.putNumber("KrakenElevator/PID slot", this.slotIndex);
        // SmartDashboard.putBoolean("KrakenElevator/use speed", !this.shouldUsePIDControl());
        // SmartDashboard.putNumber("KrakenElevator/target speed", this.speed);

        // SmartDashboard.putNumber("KrakenElevator/Motor1/speed", topRightMotor.get());
        // SmartDashboard.putNumber("KrakenElevator/Motor2/speed", bottomRightMotor.get());
        // SmartDashboard.putNumber("KrakenElevator/Motor3/speed", topLeftMotor.get());
        // SmartDashboard.putNumber("KrakenElevator/Motor4/speed", bottomLeftMotor.get());

        // SmartDashboard.putNumber("KrakenElevator/Motor1/voltage", topRightMotor.getBusVoltage());
        // SmartDashboard.putNumber("KrakenElevator/Motor2/voltage", bottomRightMotor.getBusVoltage());
        // SmartDashboard.putNumber("KrakenElevator/Motor3/voltage", topLeftMotor.getBusVoltage());
        // SmartDashboard.putNumber("KrakenElevator/Motor4/voltage", bottomLeftMotor.getBusVoltage());

        SmartDashboard.putNumber("KrakenElevator/Position", this.getPosition());
        SmartDashboard.putNumber("KrakenElevator/Velocity",
                this.m_topRightElevatorMotor.getVelocity().getValueAsDouble());
    }
}
