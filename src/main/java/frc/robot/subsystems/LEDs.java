// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashMap;

public class LEDs extends SubsystemBase {

    private static Spark m_blinkin;
    private static LEDs m_controller = null;
    private static blinkinColors m_pattern;
    
    public enum blinkinColors {
        //&PATERNS
        RAINBOW_RAINBOW (-0.99),
        COLORWAVE_RAINBOW(-0.45),
        COLORWAVE_PARTY(-0.43),
        LARSONSCANNER_RED(-0.35),
        //&SOLID COLORS
        HOT_PINK (+0.57),
        BLUE_GREEN (+0.79),
        VIOLET (+0.91),
        WHITE (+0.93),
        GREEN (+0.77);
        //?What does this do?
        private final double ledValue;
        private blinkinColors(double ledValue){
            this.ledValue = ledValue;
        }
    }

  /** Creates a new LEDs. */
  public LEDs() {
    this.m_blinkin = new Spark(9);

  }

  public void setPattern(blinkinColors pattern) {
    m_pattern = pattern;
    m_blinkin.set(m_pattern.ledValue);
  }

  //?What does this do
  public blinkinColors getCurrentPattern() {
    return m_pattern;
  }

  public void idleColor(){
    setPattern(blinkinColors.COLORWAVE_RAINBOW);
  }

// TODO -- set this up
// https://docs.revrobotics.com/rev-crossover-products/blinkin/gs

public class LEDs extends SubsystemBase {
    /** Creates a new LEDs Subsystem. */
    public LEDs() {}

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
