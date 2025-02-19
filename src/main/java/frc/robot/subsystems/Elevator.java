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
import frc.robot.Constants.Settings;
import java.util.function.Supplier;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

// see:
// https://github.com/REVrobotics/REVLib-Examples/blob/9b4cd410b6cc7fa8ed96b324dd9ecf1b4a2bbfd5/Java/SPARK/Open%20Loop%20Arcade%20Drive/src/main/java/frc/robot/Robot.java

public class Elevator extends SubsystemBase {
    // private final SparkMax RelativeEncoder;
    private final SparkMax firstMotor;
    private final SparkMax secondMotor;
    private final SparkMax thirdMotor;
    private final SparkMax fourthMotor;

    // https://docs.revrobotics.com/rev-crossover-products/sensors/tbe/application-examples#brushless-motors

    /** Creates a new Elevator Subsystem. */
    public Elevator() {
        // TODO make these IDs into constants
        // looking at the elevator with the motors in view
        this.firstMotor = new SparkMax(Ports.Elevator.kCAN_ID_TOP_RIGHT, MotorType.kBrushless); // top-right
        this.secondMotor = new SparkMax(Ports.Elevator.kCAN_ID_BOTTOM_RIGHT, MotorType.kBrushless); // bottom-right (**leader**)
        this.thirdMotor = new SparkMax(Ports.Elevator.kCAN_ID_TOP_LEFT, MotorType.kBrushless); // top-left
        this.fourthMotor = new SparkMax(Ports.Elevator.kCAN_ID_BOTTOM_LEFT, MotorType.kBrushless); // bottom-left

        // TODO verify these config settings
        SparkMaxConfig globalConfig = new SparkMaxConfig();
        globalConfig.smartCurrentLimit(60).idleMode(IdleMode.kBrake);

        SparkMaxConfig firstMotorConfig = new SparkMaxConfig();
        SparkMaxConfig secondMotorConfig = new SparkMaxConfig();
        SparkMaxConfig thirdMotorConfig = new SparkMaxConfig();
        SparkMaxConfig fourthMotorConfig = new SparkMaxConfig();

        // TODO verify these config settings
        firstMotorConfig.apply(globalConfig).follow(secondMotor);
        secondMotorConfig.apply(globalConfig);
        thirdMotorConfig.apply(globalConfig).inverted(Settings.Elevator.TOP_LEFT_INVERT)
                .follow(secondMotor);
        fourthMotorConfig.apply(globalConfig).inverted(Settings.Elevator.BOTTOM_LEFT_INVERT)
                .follow(secondMotor);


        // firstMotorConfig.alternateEncoder.apply(new
        // AlternateEncoderConfig().countsPerRevolution(8192));
        // TODO how do we want to configure the through-boro encoder?
        // https://docs.revrobotics.com/brushless/spark-max/encoders/alternate-encoder

        firstMotor.configure(firstMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        secondMotor.configure(secondMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        thirdMotor.configure(thirdMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        fourthMotor.configure(fourthMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

    }

    public Command up() {
        // TODO invert this in testing if needed
        return this.applySpeed(() -> 0.5);
    }

    public Command down() {
        // TODO invert this in testing if needed
        return this.applySpeed(() -> -0.5);
    }

    public Command stop() {
        return this.applySpeed(() -> 0.);
    }

    public Command applySpeed(Supplier<Double> speed) {
        if (speed == null) {
            return Commands.none();
        }
        // SubsystemBase.runOnce implicitly requires `this` subsystem.
        return this.runOnce(() -> {
            this.secondMotor.set(speed.get());
        });
    }

    public Command L1() {
        return this.runOnce(() -> {
            // TODO this should put the elevator in the L1 position
            DriverStation.reportWarning("Please implement me!", Thread.currentThread().getStackTrace());
        });
    }

    public Command L2() {
        return this.runOnce(() -> {
            // TODO this should put the elevator in the L2 position
            DriverStation.reportWarning("Please implement me!", Thread.currentThread().getStackTrace());
        });
    }

    public Command L3() {
        return this.runOnce(() -> {
            // TODO this should put the elevator in the L3 position
            DriverStation.reportWarning("Please implement me!", Thread.currentThread().getStackTrace());
        });
    }

    public Command L4() {
        return this.runOnce(() -> {
            // TODO this should put the elevator in the L4 position
            DriverStation.reportWarning("Please implement me!", Thread.currentThread().getStackTrace());
        });
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Elevator Speed", this.secondMotor.get());
    }
}
