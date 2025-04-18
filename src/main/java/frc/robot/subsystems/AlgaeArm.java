// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Settings;

public class AlgaeArm extends SubsystemBase {

    private final SparkMax leftMotor = new SparkMax(Settings.AlgaeFloorIntake.CAN.ID_ARM,
            Settings.AlgaeFloorIntake.ALGAE_FLOOR_INTAKE_MOTORTYPE);
    private final RelativeEncoder encoder = this.leftMotor.getEncoder();
    private final SparkClosedLoopController controller = leftMotor.getClosedLoopController();


    public AlgaeArm() {
        configureMotors();
    }

    public void configureMotors() {
        SparkMaxConfig globalConfig = new SparkMaxConfig();
        SparkMaxConfig leftMotorConfig = new SparkMaxConfig();

        globalConfig.smartCurrentLimit(40).idleMode(IdleMode.kBrake).inverted(true);

        leftMotorConfig.apply(globalConfig).closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder).maxOutput(.8).p(1);

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
        this.leftMotor.set(-0.2);
    }

    public void armRotateCCW() {
        this.leftMotor.set(0.2);
    }


    public void runRollerCCW() {
        // this.rollerMotor.set(-1);
    }

    public void stopRoller() {
        // this.rollerMotor.set(0); 
    }

    public void setReference(double setpoint) {
        this.controller.setReference(setpoint, ControlType.kPosition);
    }

    public void setSpeed(double setpoint) {
        this.leftMotor.set(setpoint);
    }

    public void stopArm() {
        this.setSpeed(0);
    }

    public void resetArmPosition() {
        System.out.printf("Resetting Algae Arm encoder position (%f) -> zero\n",
                this.getPosition());
        this.encoder.setPosition(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Spoiler/Arm/Position", this.encoder.getPosition());
        SmartDashboard.putNumber("Spoiler/Arm/Velocity", this.encoder.getVelocity());
    }
}
