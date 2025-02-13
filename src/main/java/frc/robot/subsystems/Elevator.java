// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
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

    private final SparkMaxConfig firstMotorConfig;
    private final SparkMaxConfig secondMotorConfig;
    private final SparkMaxConfig thirdMotorConfig;
    private final SparkMaxConfig fourthMotorConfig;

    /*
     Documentation Referenced: 
     https://docs.revrobotics.com/brushless/spark-max/encoders/alternate-encoder
     https://docs.revrobotics.com/rev-crossover-products/sensors/tbe/application-examples

     According to this documentation, the through-bore encoder is interpreted as an alternate encoder via code. So, we must use 
     an alternateencoder configuration. However, when we retrieve the through bore encoder from the SparkMAX it is connected to,
     it returns a relative encoder object, hence why it is declared as such below.
     */

     // ! PLEASE LEAVE THESE COMMENTED, IF AN ENCODER IS DECLARED BUT IS NOT CONNECTED IT WILL RETURN AN ERROR
    // private final RelativeEncoder throughBoreEncoder;
    // private final AlternateEncoderConfig alternateEncoderConfig;

    /** Creates a new Elevator Subsystem. */
    public Elevator() {
        // TODO make these IDs into constants
        // looking at the elevator with the motors in view

        this.firstMotor = new SparkMax(1, MotorType.kBrushless); // top-right (**leader**)
        this.secondMotor = new SparkMax(2, MotorType.kBrushless); // bottom-right
        this.thirdMotor = new SparkMax(3, MotorType.kBrushless); // top-left
        this.fourthMotor = new SparkMax(4, MotorType.kBrushless); // bottom-left

        // ! PLEASE LEAVE THIS LINE COMMENTED FOR THE SAME REASON
        // this.throughBoreEncoder = firstMotor.getAlternateEncoder();

        /*
        Instead of setting a global configuration, I want to try setting these individually. I am unsure if the global configurations 
        were overriding correctly. Perahps if we configure each motor seperately?
         */

        // TODO verify these config settings
        // && FIRST MOTOR CONFIGURATION 
        firstMotorConfig = new SparkMaxConfig();
        firstMotorConfig.idleMode(IdleMode.kBrake);
        firstMotorConfig.smartCurrentLimit(60);

        // && SECOND MOTOR CONFIG
        secondMotorConfig = new SparkMaxConfig();
        secondMotorConfig.idleMode(IdleMode.kBrake);
        secondMotorConfig.smartCurrentLimit(60);
        secondMotorConfig.follow(firstMotor);

        //&& THIRD MOTOR CONFIG
        thirdMotorConfig = new SparkMaxConfig();
        thirdMotorConfig.idleMode(IdleMode.kBrake);
        thirdMotorConfig.smartCurrentLimit(60);
        thirdMotorConfig.inverted(true);
        thirdMotorConfig.follow(firstMotor);

        //&& FOURTH MOTOR CONFIG
        fourthMotorConfig = new SparkMaxConfig();
        fourthMotorConfig.idleMode(IdleMode.kBrake);
        fourthMotorConfig.smartCurrentLimit(60);
        fourthMotorConfig.inverted(true);
        fourthMotorConfig.follow(firstMotor);

        // firstMotorConfig.alternateEncoder.apply(new
        // AlternateEncoderConfig().countsPerRevolution(8192));
        // TODO how do we want to configure the through-boro encoder?
        // https://docs.revrobotics.com/brushless/spark-max/encoders/alternate-encoder

        /* 
         Here, the configurations are applied to the motors individually.
         */
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
        return this.applySpeed(0.5);
    }

    public Command down() {
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
