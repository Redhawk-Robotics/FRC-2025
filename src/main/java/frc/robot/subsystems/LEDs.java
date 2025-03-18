// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Settings;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;


public class LEDs extends SubsystemBase {
    private final CANdle m_candle;
    private final int LEDCount = Settings.LEDs.LED_COUNT;
    
    private Animation m_toAnimate = null;

  /** Creates a new LEDs. */
  public LEDs() {
    this.m_candle = new CANdle(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
