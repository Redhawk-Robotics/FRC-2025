// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;
import java.util.function.Supplier;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AlternateEncoderConfig;
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

    // TODO
    // private final SparkClosedLoopController controller;

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

        // TODO verify these global config settings
        SparkMaxConfig globalConfig = new SparkMaxConfig();
        globalConfig.smartCurrentLimit(60).idleMode(IdleMode.kBrake);

        SparkMaxConfig topRightMotorConfig = new SparkMaxConfig();
        SparkMaxConfig bottomRightMotorConfig = new SparkMaxConfig();
        SparkMaxConfig topLeftMotorConfig = new SparkMaxConfig();
        SparkMaxConfig bottomLeftMotorConfig = new SparkMaxConfig();

        // TODO verify these per-motor config settings
        topRightMotorConfig.apply(globalConfig).follow(bottomRightMotor, false);
        bottomRightMotorConfig.apply(globalConfig); // leader
        topLeftMotorConfig.apply(globalConfig).follow(bottomRightMotor, true);
        bottomLeftMotorConfig.apply(globalConfig).follow(bottomRightMotor, true);

        // TODO
        // if we plan on running the bottom-right motor with a through-bore encoder
        // configured with the alternate-encoder adapter
        // <https://docs.revrobotics.com/rev-crossover-products/sensors/tbe/application-examples#brushless-motors>
        // then use this configuration
        // see also <https://docs.revrobotics.com/brushless/spark-max/encoders/alternate-encoder>
        //
        // bottomRightMotorConfig.alternateEncoder// leader config
        //         .setSparkMaxDataPortConfig()// required
        //         .countsPerRevolution(8192); // TODO correct value?
        //
        // TODO
        // make sure the closed-loop configuration is correct
        // we should use MAXMotion:
        // <https://docs.revrobotics.com/revlib/spark/closed-loop/closed-loop-control-getting-started#maxmotion-parameters>
        // for tuning PID see
        // <https://docs.revrobotics.com/revlib/spark/closed-loop/getting-started-with-pid-tuning#tuning>
        //
        // bottomRightMotorConfig.closedLoop// configure closed-loop PID control
        //         .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)//
        //         .pid(0.5, 0, 0)// TODO tune PID constants -- no kF
        //                 .maxMotion//
        //                         .maxVelocity(0)//
        //                         .maxAcceleration(0)//
        //                         .allowedClosedLoopError(0);

        topRightMotor.configure(topRightMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        bottomRightMotor.configure(bottomRightMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        topLeftMotor.configure(topLeftMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        bottomLeftMotor.configure(bottomLeftMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // TODO use this
        // this.controller = bottomRightMotor.getClosedLoopController();
    }

    public Command up() {
        // TODO invert this in testing if needed
        return this.applySpeedRequest(() -> 0.5);
    }

    public Command down() {
        // TODO invert this in testing if needed
        return this.applySpeedRequest(() -> -0.5);
    }

    public Command stop() {
        return this.applySpeedRequest(() -> 0.);
    }

    public Command applySpeedRequest(Supplier<Double> speed) {
        if (speed == null) {
            return Commands.none();
        }
        // SubsystemBase.runOnce implicitly requires `this` subsystem.
        return this.runOnce(() -> {
            this.bottomRightMotor.set(speed.get());
        });
    }

    private Command goToPosition(String name, double position, double error) {
        return this.runOnce(() -> {
            // TODO this should put the elevator in the given position
            DriverStation.reportWarning("Please implement me!",
                    Thread.currentThread().getStackTrace());
            // EG:
            // this.controller.setReference(position, ControlType.kMAXMotionPositionControl);
            // do not break from this command until the elevator is at the position
        }).andThen(Commands.run(() -> { // should run repeatedly
            System.out.printf("Waiting for Elevator to get to position %f for '%s'...\n", position,
                    name);
            try {
                // TODO -- ensure that this does not block the robot
                // from doing anything else. It _shouldn't_, but check to be sure
                wait(1000); // wait 1000ms before printing again
            } catch (InterruptedException e) {
                // the command was interrupted while waiting
                // OK to ignore
            }
        }).until(() -> {
            // return when the Elevator is in/near the correct position
            // EG:
            //
            // return Math.abs(//
            // this.bottomRightMotor.getAlternateEncoder().getPosition() - position) < error;

            return true;
        })).andThen(() -> {
            System.out.printf("Elevator at position '%s'!\n", name);
        });
    }

    public Command L1() {
        // EG, when ready:
        // return this.goToPosition("L1", 3, 0.1); // tune these!

        return this.runOnce(() -> {
            // TODO this should put the elevator in the L1 position
            DriverStation.reportWarning("Please implement me!",
                    Thread.currentThread().getStackTrace());
            // do not break from this command until the elevator is at the position
        });
    }

    public Command L2() {
        return this.runOnce(() -> {
            // TODO this should put the elevator in the L2 position
            DriverStation.reportWarning("Please implement me!",
                    Thread.currentThread().getStackTrace());
            // do not break from this command until the elevator is at the position
        });
    }

    public Command L3() {
        return this.runOnce(() -> {
            // TODO this should put the elevator in the L3 position
            DriverStation.reportWarning("Please implement me!",
                    Thread.currentThread().getStackTrace());
            // do not break from this command until the elevator is at the position
        });
    }

    public Command L4() {
        return this.runOnce(() -> {
            // TODO this should put the elevator in the L4 position
            DriverStation.reportWarning("Please implement me!",
                    Thread.currentThread().getStackTrace());
            // do not break from this command until the elevator is at the position
        });
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Elevator Speed", this.bottomRightMotor.get());
        // TODO put other relevant values, eg
        // bottomRightMotor.getAlternateEncoder().getPosition();
    }
}
