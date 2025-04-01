// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.util.Color;



public class LEDs extends SubsystemBase {
    private final Spark m_blinkin;
    private double m_color = 0.0;

    public enum colorModes {
        CONFETTI(-0.87),
        RAINBOW(-0.99),

        VIOLET(0.91),
        PINK(0.57),
        GREEN(0.77 ),
        RED(0.61),
        YELLOW(0.69);

        private double v = 0.0;

        private colorModes(double v){
            this.v = v;
        }
    }
  /** Creates a new LEDs. */
  public LEDs() {
    this.m_blinkin = new Spark(0);
     SmartDashboard.putNumber("LEDController/color", m_color);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_blinkin.set(m_color);
  }
}
