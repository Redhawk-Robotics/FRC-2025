// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
        this.firstMotor = new SparkMax(1, MotorType.kBrushless); // top-right (** right leader**)
        this.secondMotor = new SparkMax(2, MotorType.kBrushless); // bottom-right
        this.thirdMotor = new SparkMax(3, MotorType.kBrushless); // top-left ( ** left leader **)
        this.fourthMotor = new SparkMax(4, MotorType.kBrushless); // bottom-left

        // TODO verify these config settings
        SparkMaxConfig globalConfig = new SparkMaxConfig();
        globalConfig.smartCurrentLimit(60).idleMode(IdleMode.kBrake);

        SparkMaxConfig firstMotorConfig = new SparkMaxConfig();
        SparkMaxConfig secondMotorConfig = new SparkMaxConfig();
        SparkMaxConfig thirdMotorConfig = new SparkMaxConfig();
        SparkMaxConfig fourthMotorConfig = new SparkMaxConfig();

        /*
         I would find having only one leader to be more preferable, but I also want to see if this works. 
         */

        // TODO verify these config settings
        firstMotorConfig.apply(globalConfig);
        secondMotorConfig.apply(globalConfig).follow(firstMotor);

        thirdMotorConfig.apply(globalConfig).inverted(true);
        fourthMotorConfig.apply(globalConfig).inverted(true).follow(secondMotor);


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
        return this.applySpeed(0.5);
    }

    public Command down() {
        // TODO invert this in testing if needed
        return this.applySpeed(-0.5);
    }

    public Command stop() {
        return this.applySpeed(0);
    }

    private Command applySpeed(double speed) {
        // SubsystemBase.runOnce implicitly requires `this` subsystem.
        return this.runOnce(() -> {
            this.firstMotor.set(speed);
        });
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
