// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;
import frc.robot.Constants.Settings;

public class AlgaeFloorIntake extends SubsystemBase {
  /** Creates a new algaeFloorIntake. */
  
    private final SparkMax leftMotor = new SparkMax(Ports.AlgaeFloorIntake.kCAN_ID_LEFT, Settings.AlgaeFloorIntake.ALGAE_FLOOR_INTAKE_MOTORTYPE);
    private final SparkMax rollerMotor = new SparkMax(Ports.AlgaeFloorIntake.kCAN_ID_ROLLER, Settings.AlgaeFloorIntake.ALGAE_FLOOR_INTAKE_MOTORTYPE);
    
    private final SparkAbsoluteEncoder encoder = leftMotor.getAbsoluteEncoder();
    private final SparkClosedLoopController controller = leftMotor.getClosedLoopController();
    

    public enum positions {
        INSIDE(0),
        OUTSIDE(5);

        private final double pos;
        private positions(double p) {
            this.pos = p;
        }
        public double getpos() {
            return this.pos;
        }
    }

  public AlgaeFloorIntake() {
    configureMotors();

  }

  public void configureMotors() {
    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
    SparkMaxConfig rollerMotorConfig = new SparkMaxConfig();
    
    globalConfig.smartCurrentLimit(60).idleMode(IdleMode.kBrake);
    
    rollerMotorConfig.apply(globalConfig).inverted(true);

    leftMotorConfig.apply(globalConfig).closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder); // NO PID YET
  }

  public void setReferencePosition(){

  }

  public double getPosition() {
    return this.encoder.getPosition();
  }

  public double getVelocity() {
    return this.encoder.getPosition();
  }

  public Command armRotateCW() {
    return this.runOnce(
        () -> { this.leftMotor.set(0.2);}
    );
  }

  public Command armRotateCCW() {
    return this.runOnce(
        () -> { this.leftMotor.set(-0.2);}
    );
  }

  public Command runRollerCW() {
    return this.runOnce(
        () -> { this.rollerMotor.set(1);}
    );
  }

  public Command runRollerCCW(){
    return this.runOnce(
        () -> { this.rollerMotor.set(-1); }
    );
  }

  public Command stopRoller(){
    return this.runOnce(
        () -> { this.rollerMotor.set(0); }
    );
  }

  public void moveToSetPoint( double setpoint) {
    controller.setReference(setpoint, ControlType.kPosition);
  }

  public Command intakeAlgae() {
    return Commands.parallel(
        this.runOnce(() -> moveToSetPoint(positions.OUTSIDE.getpos())),
        runRollerCCW()
    );
  }

  public Command returnHome(){
    return Commands.parallel(
        this.runOnce( () -> { moveToSetPoint(positions.INSIDE.pos); }),
        stopRoller()
    );
  }

  public Command outtakeAlgae() {
    return Commands.parallel(
        this.runOnce( ()-> moveToSetPoint(positions.OUTSIDE.getpos())),
        runRollerCW()
    );
  }

  // TODO MAKE A COMMAND THAT TAKES AN ENUM POSITION FOR AN INPUT, THIS CONTROLS THE OUTTAKE POSITION

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Floor Algae Intkae/ Encoder", getPosition());
    SmartDashboard.putNumber("Intake Voltage", rollerMotor.getBusVoltage());
  }
}
