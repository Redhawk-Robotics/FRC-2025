// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeFloorIntakeComponents;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.Settings;

public class AlgaeFloorIntakeArm {

    private final SparkMax leftMotor = new SparkMax(Settings.AlgaeFloorIntake.CAN.ID_ARM,
            Settings.AlgaeFloorIntake.ALGAE_FLOOR_INTAKE_MOTORTYPE);
    private final SparkAbsoluteEncoder encoder = leftMotor.getAbsoluteEncoder();
    private final SparkClosedLoopController controller = leftMotor.getClosedLoopController();

    public enum positions {
        INSIDE(0), OUTSIDE(5);

        private final double pos;

        private positions(double p) {
            this.pos = p;
        }

        public double getpos() {
            return this.pos;
        }
    }

    public AlgaeFloorIntakeArm() {
        configureMotors();
    }

    public void configureMotors() {
        SparkMaxConfig globalConfig = new SparkMaxConfig();
        SparkMaxConfig leftMotorConfig = new SparkMaxConfig();

        globalConfig.smartCurrentLimit(60).idleMode(IdleMode.kBrake);

        leftMotorConfig.apply(globalConfig);
        // leftMotorConfig.apply(globalConfig).closedLoop
        //         .feedbackSensor(FeedbackSensor.kPrimaryEncoder).maxOutput(.2); // NO PID YET

        leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public double getPosition() {
        return this.encoder.getPosition();
    }

    public double getVelocity() {
        return this.encoder.getPosition();
    }

    public void armRotateCW() {
        this.leftMotor.set(0.2);
    }

    public void armRotateCCW() {
        this.leftMotor.set(-0.2);
    }


    public void runRollerCCW() {
        // this.rollerMotor.set(-1);
    }

    public void stopRoller() {
        // this.rollerMotor.set(0); 
    }

    public void setRef(double setpoint) {
        this.controller.setReference(setpoint, ControlType.kPosition);
    }

    public void setSpeed(double setpoint) {
        this.leftMotor.set(setpoint);
        // this.controller.setReference(setpoint, ControlType.kDutyCycle);
    }
}
