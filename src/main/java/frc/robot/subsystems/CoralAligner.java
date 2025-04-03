// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Settings;

public class CoralAligner extends SubsystemBase {
  /** Creates a new coralAligner. */

  // I made a seperate subsystem for these things because I don't want to touch swerve
  private CANrange m_leftCanRange;
  private CANrange m_rightCanRange;

  private double m_leftDesired, rightDesired;

  private double m_leftDistance, m_rightDistance;

  public CoralAligner() {
    configure();
  }

  public void configure(){

    m_leftCanRange = new CANrange(Settings.CoralAligner.CAN.kLEFT, Settings.CoralAligner.CAN.kBUS);
    m_leftCanRange = new CANrange(Settings.CoralAligner.CAN.kRIGHT, Settings.CoralAligner.CAN.kBUS);
    CANrangeConfiguration config = new CANrangeConfiguration();

    //TODO What config settings do we need?

    m_leftCanRange.getConfigurator().apply(config);
    m_rightCanRange.getConfigurator().apply(config);

  }

  public void updateDistances() {
    m_leftDistance = m_leftCanRange.getDistance().getValueAsDouble();
    m_rightDistance = m_leftCanRange.getDistance().getValueAsDouble();
  }

  public Boolean isAlignedLeft() {
    return false;
  }

  public Boolean isAlignedRight() {
    return false;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("CANRANGE/Left Distance", m_leftDistance);
    SmartDashboard.putNumber("CANRANGE/Right Distance", m_rightDistance);


    updateDistances();
  }
}
