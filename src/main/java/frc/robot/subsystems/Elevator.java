// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;
import java.util.function.Supplier;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

// see:
// https://github.com/REVrobotics/REVLib-Examples/blob/9b4cd410b6cc7fa8ed96b324dd9ecf1b4a2bbfd5/Java/SPARK/Open%20Loop%20Arcade%20Drive/src/main/java/frc/robot/Robot.java

public class Elevator extends SubsystemBase {



    // looking at the elevator with the motors in view
    // looking at the front of the robot
    private final SparkMax topRightMotor = new SparkMax(//
            Ports.Elevator.kCAN_ID_TOP_RIGHT, MotorType.kBrushless);
    private final SparkMax bottomRightMotor = new SparkMax(// (**leader**)
            Ports.Elevator.kCAN_ID_BOTTOM_RIGHT, MotorType.kBrushless);
    private final SparkMax topLeftMotor = new SparkMax(//
            Ports.Elevator.kCAN_ID_TOP_LEFT, MotorType.kBrushless);
    private final SparkMax bottomLeftMotor = new SparkMax(//
            Ports.Elevator.kCAN_ID_BOTTOM_LEFT, MotorType.kBrushless);

    private final RelativeEncoder encoder = topRightMotor.getEncoder();
    private final SparkClosedLoopController controller = topRightMotor.getClosedLoopController();

    private double setPoint;
    private ControlType controlType;
    private int slotIndex;

    // https://docs.revrobotics.com/rev-crossover-products/sensors/tbe/application-examples#brushless-motors

    /** Creates a new Elevator Subsystem. */
    public Elevator() {
        this.configureMotors(//
                0.1, 0, 0, //
                0.050, 0, 0, //
                0.025, 0, 0);

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

        // TODO -- verify that re-configuring the motors
        // does not invalidate the two objects above
    }

    // this is only public so we can tune PID
    public void configureMotors(double kP0, double kI0, double kD0, double kP1, double kI1,
            double kD1, double kP2, double kI2, double kD2) {
        System.out.printf(
                "Configuring Elevator motors (kP:%f kI:%f kD:%f)(kP:%f kI:%f kD:%f)(kP:%f kI:%f kD:%f)...\n",
                kP0, kI0, kD0, kP1, kI1, kD1, kP2, kI2, kD2);

        // TODO verify these global config settings
        SparkMaxConfig globalConfig = new SparkMaxConfig();
        globalConfig.smartCurrentLimit(60).idleMode(IdleMode.kBrake);

        SparkMaxConfig topRightMotorConfig = new SparkMaxConfig();
        SparkMaxConfig bottomRightMotorConfig = new SparkMaxConfig();
        SparkMaxConfig topLeftMotorConfig = new SparkMaxConfig();
        SparkMaxConfig bottomLeftMotorConfig = new SparkMaxConfig();

        // TODO verify these per-motor config settings
        topRightMotorConfig.apply(globalConfig);
        bottomRightMotorConfig.apply(globalConfig).follow(topRightMotor, false); // leader
        topLeftMotorConfig.apply(globalConfig).follow(bottomRightMotor, true);
        bottomLeftMotorConfig.apply(globalConfig).follow(bottomRightMotor, true);

        // TODO
        // if we plan on running the bottom-right motor with a through-bore encoder
        // configured with the alternate-encoder adapter
        // <https://docs.revrobotics.com/rev-crossover-products/sensors/tbe/application-examples#brushless-motors>
        // then use this configuration
        // see also <https://docs.revrobotics.com/brushless/spark-max/encoders/alternate-encoder>
        //

        // TODO kill
        // elevator max == 32
        double conversionFactor = 100. / 32.;
        topRightMotorConfig.encoder.positionConversionFactor(conversionFactor)
                .velocityConversionFactor(conversionFactor);

        // double zeroOffset = 0.13;
        // // double conversionFactor = Inches.of(74).minus(Inches.of(43.5)).magnitude();
        // double conversionFactor = 150.;
        // topRightMotorConfig.absoluteEncoder// leader config
        //         .setSparkMaxDataPortConfig()//
        //         .inverted(true)// positive == go up
        //         .zeroOffset(zeroOffset)// TODO
        //         .positionConversionFactor(conversionFactor)// inch/rev (max-min)
        //         .velocityConversionFactor(conversionFactor);

        // TODO
        // make sure the closed-loop configuration is correct
        // we should use MAXMotion:
        // <https://docs.revrobotics.com/revlib/spark/closed-loop/closed-loop-control-getting-started#maxmotion-parameters>
        // for tuning PID see
        // <https://docs.revrobotics.com/revlib/spark/closed-loop/getting-started-with-pid-tuning#tuning>
        //

        topRightMotorConfig.closedLoop// configure closed-loop PID control
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)//
                .pid(kP0, kI0, kD0, ClosedLoopSlot.kSlot0)// no kF
                .pid(kP1, kI1, kD1, ClosedLoopSlot.kSlot1)// no kF
                .pid(kP2, kI2, kD2, ClosedLoopSlot.kSlot2)// no kF
        ;
        // slot 0 == velocity control
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

    public Command setReferenceRequest(Supplier<Double> reference, ControlType type) {
        if (reference == null) {
            DriverStation.reportError(
                    "Elevator.setReferenceRequest received null reference supplier",
                    Thread.currentThread().getStackTrace());
            return Commands.none();
        }
        return this.runOnce(() -> {
            this.setPoint = reference.get();
            this.controlType = type;
            // if velocity control, use slot 0
            // else if want - current > 0 then go up, so use slot 1
            // otherwise use slot 2
            this.slotIndex = type == ControlType.kVelocity ? 0
                    : (reference.get() - this.getPosition() > 0 ? 1 : 2);
        });
    }

    // Just for testing you can delete later
    public void stopElevator() {
        this.topRightMotor.set(0.0);
        this.bottomLeftMotor.set(0.0);
        this.bottomRightMotor.set(0.0);
        this.topLeftMotor.set(0.0);
    }

    // Might be useful to have this
    public void resetElevatorPosition() {
        this.encoder.setPosition(0.0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        if (this.controller != null) {
            this.controller.setReference(this.setPoint, this.controlType,
                    ClosedLoopSlot.values()[slotIndex]);
        }

        SmartDashboard.putNumber("Elevator/set point", this.setPoint);
        SmartDashboard.putString("Elevator/control type", this.controlType.name());
        SmartDashboard.putNumber("Elevator/slot", this.slotIndex);

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
