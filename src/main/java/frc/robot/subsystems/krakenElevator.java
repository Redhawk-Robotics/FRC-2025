// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Settings;

//An elevator created with two Krakens instead of Sparkmaxes.

//https://v6.docs.ctr-electronics.com/en/latest/docs/migration/migration-guide/control-requests-guide.html

public class krakenElevator extends SubsystemBase {
  /** Creates a new krakenElevator. */
  private final TalonFX m_leftElevatorMotor = new TalonFX(Settings.Elevator.CAN.ID_TOP_LEFT);
  private final TalonFX m_rightElevatorMotor = new TalonFX(Settings.Elevator.CAN.ID_TOP_RIGHT);

  //This doesn't recognize itself as a class if I put it in configs..? Something about scope, maybe. Java!!!!!
  final VelocityVoltage m_velocity = new VelocityVoltage(0);


  public krakenElevator() {
    configureMotors( Settings.Elevator.kP_UP, Settings.Elevator.kI_UP, Settings.Elevator.kD_UP, //
    Settings.Elevator.kP_DOWN, Settings.Elevator.kI_DOWN, Settings.Elevator.kD_DOWN);
  }

  public void configureMotors(double kP1, double kI1, double kD1, double kP2, double kI2,
  double kD2){

    //I am thinking that velocity would be the easiest to get fully tuned in like 2 days

    TalonFXConfiguration m_rightElevatorMotorConfigs = new TalonFXConfiguration();
    var slot0configs = new Slot0Configs();

    //https://v6.docs.ctr-electronics.com/en/latest/docs/migration/migration-guide/closed-loop-guide.html

    //Setting Slot 0 Gains
    slot0configs.kV = 0;
    slot0configs.kP = kP1;
    slot0configs.kI = kI1;
    slot0configs.kD = kD1;

    // m_rightElevatorMotorConfigs.Slot0.kP = 0;
    // m_rightElevatorMotorConfigs.Slot0.kI = 0;
    // m_rightElevatorMotorConfigs.Slot0.kD = 0;

    //Volts need to be confirmed
    m_rightElevatorMotorConfigs.Voltage.withPeakForwardVoltage(0)
    .withPeakReverseVoltage(0);

    //Amps need to be confirmed
    m_rightElevatorMotorConfigs.TorqueCurrent.withPeakForwardTorqueCurrent(0)
        .withPeakReverseTorqueCurrent(0);

    //Make sure the encoder is set at 0 on init
    m_leftElevatorMotor.setPosition(0);

    m_rightElevatorMotor.getConfigurator().apply(slot0configs, 0.5);
    m_rightElevatorMotor.setControl(m_velocity.withVelocity(50));
    m_leftElevatorMotor.setControl(new Follower(Settings.Elevator.CAN.ID_TOP_RIGHT, true));


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
