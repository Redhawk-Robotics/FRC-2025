// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;
import java.util.function.Supplier;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
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

    private final SparkMax topRightMotor;
    private final SparkMax bottomRightMotor;
    private final SparkMax topLeftMotor;
    private final SparkMax bottomLeftMotor;

    private final RelativeEncoder encoder;
    private final SparkClosedLoopController controller;

    // https://docs.revrobotics.com/rev-crossover-products/sensors/tbe/application-examples#brushless-motors

    /** Creates a new Elevator Subsystem. */
    public Elevator() {
        // looking at the elevator with the motors in view
        // looking at the front of the robot
        this.topRightMotor = new SparkMax(//
                Ports.Elevator.kCAN_ID_TOP_RIGHT, MotorType.kBrushless);
        this.bottomRightMotor = new SparkMax(// (**leader**)
                Ports.Elevator.kCAN_ID_BOTTOM_RIGHT, MotorType.kBrushless);
        this.topLeftMotor = new SparkMax(//
                Ports.Elevator.kCAN_ID_TOP_LEFT, MotorType.kBrushless);
        this.bottomLeftMotor = new SparkMax(//
                Ports.Elevator.kCAN_ID_BOTTOM_LEFT, MotorType.kBrushless);

        this.configureMotors(0.1, 0, 0);

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
        this.encoder = topRightMotor.getEncoder();
        this.controller = topRightMotor.getClosedLoopController();
        // TODO -- verify that re-configuring the motors
        // does not invalidate the two objects above
    }

    // this is only public so we can tune PID
    public void configureMotors(double kP, double kI, double kD) {
        System.out.printf("Configuring Elevator motors (kP:%f kI:%f kD:%f)...\n", kP, kI, kD);

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
                .pid(kP, kI, kD)// TODO tune PID constants -- no kF
        ;
        // .maxMotion//
        //         .maxVelocity(100)// ??
        //         .maxAcceleration(3)//
        //         .allowedClosedLoopError(5);

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

    public Command applySpeedRequest(Supplier<Double> speed) {
        if (speed == null) {
            DriverStation.reportWarning(
                    "Elevator applySpeedRequest received null position supplier",
                    Thread.currentThread().getStackTrace());
            return Commands.none();
        }
        // SubsystemBase.runOnce implicitly requires `this` subsystem.
        return this.runOnce(() -> {
            this.topRightMotor.set(speed.get());
        });
    }

    public Command applyAddPositionRequest(Supplier<Double> position) {
        if (position == null) {
            DriverStation.reportWarning(
                    "Elevator applyAddPositionRequest received null position supplier",
                    Thread.currentThread().getStackTrace());
            return Commands.none();
        }
        // SubsystemBase.runOnce implicitly requires `this` subsystem.
        return this.runOnce(() -> {
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
