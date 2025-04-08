// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Settings;

//An elevator created with two Krakens instead of Sparkmaxes.

//https://v6.docs.ctr-electronics.com/en/latest/docs/migration/migration-guide/control-requests-guide.html

public class krakenElevator extends SubsystemBase {
  /** Creates a new krakenElevator. */
  private final TalonFX m_leftElevatorMotor = new TalonFX(Settings.Elevator.CAN.ID_TOP_LEFT);
  private final TalonFX m_rightElevatorMotor = new TalonFX(Settings.Elevator.CAN.ID_TOP_RIGHT);

  private double setPoint = 0;
  private int slotIndex = 01;
  private double speed = 0;

  public krakenElevator() {
    this.configureMotors(//
    Settings.Elevator.kP_UP, Settings.Elevator.kI_UP, Settings.Elevator.kD_UP, //
    Settings.Elevator.kP_DOWN, Settings.Elevator.kI_DOWN, Settings.Elevator.kD_DOWN);
    // this.resetElevatorPosition(); not written yet
  }

  // Parameters take in values to be configured
  public void configureMotors(double kP1, double kI1, double kD1, double kP2, double kI2,
            double kD2) {
    m_rightElevatorMotor.setControl(new Follower(Settings.Elevator.CAN.ID_TOP_LEFT, true));
    TalonFXConfiguration m_rightMotorConfigs = new TalonFXConfiguration();

    // Slot configurations 
    m_rightMotorConfigs.Slot0.kP = kP1;
    m_rightMotorConfigs.Slot0.kI = kI1;
    m_rightMotorConfigs.Slot0.kD = kD1;

    m_rightMotorConfigs.Slot1.kP = kP2;
    m_rightMotorConfigs.Slot1.kI = kI2;
    m_rightMotorConfigs.Slot1.kD = kD2;

    // slot 1 == position control, elevator up
    // slot 2 == position control, elevator down

    //Volts need to be confirmed
    m_rightMotorConfigs.Voltage.withPeakForwardVoltage(0)
    .withPeakReverseVoltage(0);

    //Amps need to be confirmed
    m_rightMotorConfigs.TorqueCurrent.withPeakForwardTorqueCurrent(0)
        .withPeakReverseTorqueCurrent(0);

    //Make sure the encoder is set at 0 on init
    m_rightElevatorMotor.setPosition(0);

    System.out.println("Done configuring Kraken motors.");
  }

  public double getPosition() {
    return m_rightElevatorMotor.getPosition().getValueAsDouble();
  }

  public StatusSignal<Angle> getPositionAsAngle() {
    return m_rightElevatorMotor.getPosition();
  }

  public void setReference(double reference) {
    System.out.printf("Setting Elevator reference to %f (current %f)\n", reference,
    this.getPosition());

    this.setPoint = reference;
    this.slotIndex = reference - this.getPosition() > 0 ? 0 : 1;
  }

  public void setSpeed(double speed) {
    this.speed = speed;
  }

  public void useSpeed() {
    //not sure about slot indexing since this has preset values that we can adjust
    //! empty
  }

//   private boolean shouldUsePIDControl() {}

  public void stopElevator(){
    this.useSpeed();
    this.setSpeed(0);
    this.m_rightElevatorMotor.set(0);
    this.m_leftElevatorMotor.set(0);

  }

  public void resetElevatorPosition(){
    m_rightElevatorMotor.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
