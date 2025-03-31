// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Settings;

//An elevator created with two Krakens instead of Sparkmaxes.

//https://v6.docs.ctr-electronics.com/en/latest/docs/migration/migration-guide/control-requests-guide.html

public class krakenElevator extends SubsystemBase {
  /** Creates a new krakenElevator. */
  private final TalonFX m_leftElevatorMotor = new TalonFX(Settings.Elevator.CAN.ID_TOP_LEFT);
  private final TalonFX m_rightElevatorMotor = new TalonFX(Settings.Elevator.CAN.ID_TOP_RIGHT);

  public krakenElevator() {
    configureMotors();
  }

  public void configureMotors(){
    m_rightElevatorMotor.setControl(new Follower(Settings.Elevator.CAN.ID_TOP_LEFT, true));
    TalonFXConfiguration m_leftElevatorMotorConfigs = new TalonFXConfiguration();
    m_leftElevatorMotorConfigs.Slot0.kP = 0;
    m_leftElevatorMotorConfigs.Slot0.kI = 0;
    m_leftElevatorMotorConfigs.Slot0.kD = 0;

    //Volts need to be confirmed
    m_leftElevatorMotorConfigs.Voltage.withPeakForwardVoltage(0)
    .withPeakReverseVoltage(0);

    //Amps need to be confirmed
    m_leftElevatorMotorConfigs.TorqueCurrent.withPeakForwardTorqueCurrent(0)
        .withPeakForwardTorqueCurrent(0);

    //Make sure the encoder is set at 0 on init
    m_leftElevatorMotor.setPosition(0);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
