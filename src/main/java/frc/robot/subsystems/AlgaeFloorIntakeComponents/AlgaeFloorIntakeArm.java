// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeFloorIntakeComponents;

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

public class AlgaeFloorIntakeArm extends SubsystemBase { // TODO do not extend SubsystemBase
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

  public AlgaeFloorIntakeArm() {
    configureMotors();
  }
  
  public void configureMotors() {
    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
    SparkMaxConfig rollerMotorConfig = new SparkMaxConfig();
    
    globalConfig.smartCurrentLimit(60).idleMode(IdleMode.kBrake);
    
    rollerMotorConfig.apply(globalConfig).inverted(true);

    leftMotorConfig.apply(globalConfig).closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder).maxOutput(.2); // NO PID YET
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

  public void runRollerCW() {
    this.runOnce(
        () -> { this.rollerMotor.set(1);}
    );
  }

  public void runRollerCCW() {
    this.rollerMotor.set(-1);
  }

  public void stopRoller(){
    this.rollerMotor.set(0); 
  }

  public void voidIntakeMethod() {
    moveToSetPoint(positions.OUTSIDE.getpos());
    rollerMotor.set(0.5);
  }

  public void voidMoveInsideMethod() {
    moveToSetPoint(positions.INSIDE.getpos());
    stopRoller();
  }

  public Command commandIntake() {
    // return this.runOnce( ()-> {voidIntakeMethod();});
    return Commands.parallel(
        moveToSetPoint(3),
        commandSetRollerSpeed(8)
    );
  }

  public void setRef(double setpoint) {
    this.controller.setReference(setpoint, ControlType.kPosition);
  }

  public Command commandMoveInside() {
    return this.runOnce( ()-> {voidMoveInsideMethod();});
  }

  public Command commandOutake() { 
    return this.runOnce( () -> { runRollerCW();});
 
    }// TODO CHECK THIS ON PRACTICE FIELD

  public Command commandStopRoller() { return this.runOnce( () -> { stopRoller();});}


  public Command moveToSetPoint( double setpoint) {
    return this.runOnce( () -> {controller.setReference(setpoint, ControlType.kPosition);});
  }

  public Command commandSetSpeed(double speed) {
    return this.runOnce( 
        () -> { leftMotor.set(speed); }
    );
    
  }

  public Command commandSetRollerSpeed(double speed) {
    return this.runOnce( 
        () -> { rollerMotor.set(speed); }
    );
    
  }

  // TODO MAKE A COMMAND THAT TAKES AN ENUM POSITION FOR AN INPUT, THIS CONTROLS THE OUTTAKE POSITION

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Floor Algae Intkae/ Encoder", getPosition());
    SmartDashboard.putNumber("Intake Voltage", rollerMotor.getBusVoltage());
  }
}
