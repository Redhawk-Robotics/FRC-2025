// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Settings;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

// see:
// https://github.com/REVrobotics/REVLib-Examples/blob/9b4cd410b6cc7fa8ed96b324dd9ecf1b4a2bbfd5/Java/SPARK/Open%20Loop%20Arcade%20Drive/src/main/java/frc/robot/Robot.java

public class Elevator extends SubsystemBase {

    // looking at the elevator with the motors in view
    // looking at the front of the robot
    private final SparkMax topRightMotor = new SparkMax(// (**leader**)
            Settings.Elevator.CAN.ID_TOP_RIGHT, Settings.Elevator.TOP_RIGHT_MOTORTYPE);
    private final SparkMax bottomRightMotor = new SparkMax(//
            Settings.Elevator.CAN.ID_BOTTOM_RIGHT, Settings.Elevator.BOTTOM_RIGHT_MOTORTYPE);
    private final SparkMax topLeftMotor = new SparkMax(//
            Settings.Elevator.CAN.ID_TOP_LEFT, Settings.Elevator.TOP_LEFT_MOTORTYPE);
    private final SparkMax bottomLeftMotor = new SparkMax(//
            Settings.Elevator.CAN.ID_BOTTOM_LEFT, Settings.Elevator.BOTTOM_LEFT_MOTORTYPE);

    private final RelativeEncoder encoder = topRightMotor.getEncoder();
    private final SparkClosedLoopController controller = topRightMotor.getClosedLoopController();

    private double setPoint = 0;
    private int slotIndex = -1;
    // slotIndex < 0 means to use .set(this.speed)
    // slotIndex 1 means to use PID for Elevator UP with this.setPoint
    // slotIndex 2 means to use PID for Elevator DOWN with this.setPoint
    private double speed = 0;

    // https://docs.revrobotics.com/rev-crossover-products/sensors/tbe/application-examples#brushless-motors

    /** Creates a new Elevator Subsystem. */
    public Elevator() {
        this.configureMotors(//
                Settings.Elevator.kP_UP, Settings.Elevator.kI_UP, Settings.Elevator.kD_UP, //
                Settings.Elevator.kP_DOWN, Settings.Elevator.kI_DOWN, Settings.Elevator.kD_DOWN);
        this.resetElevatorPosition();

        // no longer using thru-bore, because the measurement goes beyond 360deg
        // so we can't use it as an absolute encoder
        // we _could_ use it in quadrature (relative) mode --
        //   thru-bore is connected to:
        //   blue port 0
        //   yellow port 1
        // but I don't know how we can configure the Max's PID controller
        // to use an encoder not connected to the Max directly
        // So, we're just going to use the built-in relative encoder
        // which fine because the elevator always starts at the bottom (zero)
    }

    // this is only public so we can tune PID
    public void configureMotors(double kP1, double kI1, double kD1, double kP2, double kI2,
            double kD2) {
        System.out.printf(
                "Configuring Elevator motors\n\t(kP:%f kI:%f kD:%f)(up)\n\t(kP:%f kI:%f kD:%f)(down)...\n",
                kP1, kI1, kD1, kP2, kI2, kD2);

        SparkMaxConfig globalConfig = new SparkMaxConfig();
        globalConfig.smartCurrentLimit(40).idleMode(IdleMode.kBrake);

        SparkMaxConfig topRightMotorConfig = new SparkMaxConfig();
        SparkMaxConfig bottomRightMotorConfig = new SparkMaxConfig();
        SparkMaxConfig topLeftMotorConfig = new SparkMaxConfig();
        SparkMaxConfig bottomLeftMotorConfig = new SparkMaxConfig();

        topRightMotorConfig.apply(globalConfig).inverted(false);
        bottomRightMotorConfig.apply(globalConfig).follow(topRightMotor, false); // leader
        topLeftMotorConfig.apply(globalConfig).follow(bottomRightMotor, true);
        bottomLeftMotorConfig.apply(globalConfig).follow(bottomRightMotor, true);

        // TODO let's not use a conversion factor
        topRightMotorConfig.encoder.positionConversionFactor(Settings.Elevator.CONVERSION_FACTOR)
                .velocityConversionFactor(Settings.Elevator.CONVERSION_FACTOR);

        // TODO
        // make sure the closed-loop configuration is correct
        // we should use MAXMotion:
        // <https://docs.revrobotics.com/revlib/spark/closed-loop/closed-loop-control-getting-started#maxmotion-parameters>
        // for tuning PID see
        // <https://docs.revrobotics.com/revlib/spark/closed-loop/getting-started-with-pid-tuning#tuning>
        //

        topRightMotorConfig.closedLoop// configure closed-loop PID control
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)//
                .pid(kP1, kI1, kD1, ClosedLoopSlot.kSlot1)// no kF
                .pid(kP2, kI2, kD2, ClosedLoopSlot.kSlot2)// no kF
        ;
        // slot 1 == position control, elevator up
        // slot 2 == position control, elevator down

        topRightMotor.configure(topRightMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        bottomRightMotor.configure(bottomRightMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        topLeftMotor.configure(topLeftMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        bottomLeftMotor.configure(bottomLeftMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        System.out.println("Done configuring Elevator motors.");
    }

    public double getPosition() {
        return this.encoder.getPosition();
    }

    public void setReference(double reference) {
        System.out.printf("Setting Elevator reference to %f (current %f)\n", reference,
                this.getPosition());
        this.setPoint = reference;
        // if (want - current) > 0 then go up, so use slot 1
        // otherwise use slot 2
        this.slotIndex = reference - this.getPosition() > 0 ? 1 : 2;
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

    public void stopElevator() {
        this.useSpeed();
        this.setSpeed(0);
        // redundant
        this.topRightMotor.set(0.0);
        this.bottomLeftMotor.set(0.0);
        this.bottomRightMotor.set(0.0);
        this.topLeftMotor.set(0.0);
    }

    public void resetElevatorPosition() {
        // System.out.printf("Resetting Elevator encoder position (%f) -> zero\n", this.getPosition());
        this.encoder.setPosition(0.0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        if (this.shouldUsePIDControl()) {
            this.controller.setReference(this.setPoint, ControlType.kPosition,
                    ClosedLoopSlot.values()[slotIndex]);
        } else {
            this.topRightMotor.set(this.speed);
        }

        SmartDashboard.putBoolean("Elevator/use PID", this.shouldUsePIDControl());
        SmartDashboard.putNumber("Elevator/PID setPoint", this.setPoint);
        SmartDashboard.putNumber("Elevator/PID slot", this.slotIndex);
        SmartDashboard.putBoolean("Elevator/use speed", !this.shouldUsePIDControl());
        SmartDashboard.putNumber("Elevator/target speed", this.speed);

        SmartDashboard.putNumber("Elevator/Motor1/speed", topRightMotor.get());
        SmartDashboard.putNumber("Elevator/Motor2/speed", bottomRightMotor.get());
        SmartDashboard.putNumber("Elevator/Motor3/speed", topLeftMotor.get());
        SmartDashboard.putNumber("Elevator/Motor4/speed", bottomLeftMotor.get());

        SmartDashboard.putNumber("Elevator/Motor1/voltage", topRightMotor.getBusVoltage());
        SmartDashboard.putNumber("Elevator/Motor2/voltage", bottomRightMotor.getBusVoltage());
        SmartDashboard.putNumber("Elevator/Motor3/voltage", topLeftMotor.getBusVoltage());
        SmartDashboard.putNumber("Elevator/Motor4/voltage", bottomLeftMotor.getBusVoltage());

        SmartDashboard.putNumber("Elevator/Position", this.encoder.getPosition());
        SmartDashboard.putNumber("Elevator/Velocity", this.encoder.getVelocity());
    }
}
