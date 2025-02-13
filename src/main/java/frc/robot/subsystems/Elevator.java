// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
<<<<<<< Updated upstream
=======
import frc.robot.Constants.Ports;
import frc.robot.Constants.Settings;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
>>>>>>> Stashed changes

// TODO fill this in

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator Subsystem. */
  public Elevator() {}

<<<<<<< Updated upstream
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
=======
    // https://docs.revrobotics.com/rev-crossover-products/sensors/tbe/application-examples#brushless-motors

    /** Creates a new Elevator Subsystem. */
    public Elevator() {
        // TODO make these IDs into constants
        // looking at the elevator with the motors in view
        this.firstMotor = new SparkMax(Ports.Elevator.TOP_RIGHT, Settings.Elevator.TOP_RIGHT_MOTORTYPE); // top-right (**leader**)
        this.secondMotor = new SparkMax(Ports.Elevator.BOTTOM_RIGHT, Settings.Elevator.BOTTOM_RIGHT_MOTORTYPE); // bottom-right
        this.thirdMotor = new SparkMax(Ports.Elevator.TOP_LEFT, Settings.Elevator.TOP_LEFT_MOTORTYPE); // top-left
        this.fourthMotor = new SparkMax(Ports.Elevator.BOTTOM_LEFT, Settings.Elevator.TOP_RIGHT_MOTORTYPE); // bottom-left

        // TODO verify these config settings
        SparkMaxConfig globalConfig = new SparkMaxConfig();
        globalConfig.smartCurrentLimit(Settings.Elevator.CURRENT_LIMIT).idleMode(Settings.Elevator.IDLE_MODE);

        SparkMaxConfig firstMotorConfig = new SparkMaxConfig();
        SparkMaxConfig secondMotorConfig = new SparkMaxConfig();
        SparkMaxConfig thirdMotorConfig = new SparkMaxConfig();
        SparkMaxConfig fourthMotorConfig = new SparkMaxConfig();

        // TODO verify these config settings
        firstMotorConfig.apply(globalConfig);
        secondMotorConfig.apply(globalConfig).follow(firstMotor);
        thirdMotorConfig.apply(globalConfig).inverted(Settings.Elevator.TOP_LEFT_INVERT).follow(firstMotor);
        fourthMotorConfig.apply(globalConfig).inverted(Settings.Elevator.BOTTOM_LEFT_INVERT).follow(firstMotor);

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
>>>>>>> Stashed changes
}
