// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Settings;

// An elevator created with three Krakens instead of 4 Neo/Spark.

// https://v6.docs.ctr-electronics.com/en/latest/docs/migration/migration-guide/control-requests-guide.html

public class Elevator extends SubsystemBase {
    /** Creates a new krakenElevator. */
    private final TalonFX m_topRightElevatorMotor = new TalonFX(Settings.Elevator.CAN.ID_TOP_RIGHT); // leader
    private final TalonFX m_topLeftElevatorMotor = new TalonFX(Settings.Elevator.CAN.ID_TOP_LEFT);
    private final TalonFX m_bottomLeftElevatorMotor =
            new TalonFX(Settings.Elevator.CAN.ID_BOTTOM_LEFT);

    private final PositionTorqueCurrentFOC positionControl = new PositionTorqueCurrentFOC(0);
    private final DutyCycleOut speedControl = new DutyCycleOut(0);

    private double setPoint = 0;
    private double speed = 0;

    private boolean usePosition = false;

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
                .withStatorCurrentLimit(Amps.of(40));

        //Volts need to be confirmed
        // motorConfig.Voltage.withPeakForwardVoltage(0).withPeakReverseVoltage(0);

        //Amps need to be confirmed
        // motorConfig.TorqueCurrent.withPeakForwardTorqueCurrent(0)
        //         .withPeakReverseTorqueCurrent(0);

        this.m_topRightElevatorMotor.getConfigurator().apply(motorConfig);
        this.m_topLeftElevatorMotor.getConfigurator().apply(motorConfig);
        this.m_bottomLeftElevatorMotor.getConfigurator().apply(motorConfig);

        this.m_topRightElevatorMotor.getConfigurator().apply(
                (new TalonFXConfiguration().Slot0.withGravityType(GravityTypeValue.Elevator_Static))
                        .withKG(Settings.Elevator.kG)
                        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                        .withKS(Settings.Elevator.kS).withKP(kP1).withKI(kI1).withKD(kD1));

        this.m_topLeftElevatorMotor
                .setControl(new Follower(Settings.Elevator.CAN.ID_TOP_RIGHT, true));
        this.m_bottomLeftElevatorMotor
                .setControl(new Follower(Settings.Elevator.CAN.ID_TOP_RIGHT, true));

        System.out.println("Done configuring Kraken Elevator motors.");
    }


    public double getPosition() {
        return this.m_topRightElevatorMotor.getPosition().getValueAsDouble();
    }

    public void setReference(double reference) {
        // System.out.printf("Setting Elevator reference to %f (current %f)\n", reference,
        //         this.getPosition());

        this.setPoint = reference;
        this.usePosition = true;
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public void useSpeed() {
        this.usePosition = false;
    }

    public void stopElevator() {
        this.useSpeed();
        this.setSpeed(0);
    }

    public void resetElevatorPosition() {
        this.m_topRightElevatorMotor.setPosition(0);
    }

    @Override
    public void periodic() {
        if (this.usePosition) {
            this.m_topRightElevatorMotor
                    .setControl(this.positionControl.withPosition(this.setPoint));
        } else {
            this.m_topRightElevatorMotor.setControl(this.speedControl.withOutput(this.speed));
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
