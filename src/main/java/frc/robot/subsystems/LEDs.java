// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


// TODO -- set this up
// https://docs.revrobotics.com/rev-crossover-products/blinkin/gs


public class LEDs extends SubsystemBase {
    /** Creates a new LEDs Subsystem. */
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_buffer;
    private final AddressableLEDBufferView m_left;
    private final AddressableLEDBufferView m_right;
    private final LEDPattern m_red;
    private final LEDPattern m_lightPink;
    private final LEDPattern m_pink;
    private final LEDPattern m_rainbow;
    private final LEDPattern m_violet;
    private final LEDPattern m_green;
    private final LEDPattern m_white;
    private final LEDPattern m_purpleGradient;
    private final LEDPattern m_redGradient;
    private final LEDPattern m_greenBase;
    private final LEDPattern m_greenPattern;
    private final LEDPattern m_sycned;



    public LEDs() {
        this.m_led = new AddressableLED(9);
        this.m_buffer = new AddressableLEDBuffer(120);
        m_led.setLength(m_buffer.getLength());
        m_led.start();

        this.m_left = m_buffer.createView(0, 59);
        this.m_right = m_buffer.createView(60, 119).reversed();

        //SOlID COLORS

        this.m_lightPink = LEDPattern.solid(Color.kLightPink);
        this.m_pink = LEDPattern.solid(Color.kPink);
        this.m_red = LEDPattern.solid(Color.kRed);
        this.m_violet = LEDPattern.solid(Color.kViolet);
        this.m_green = LEDPattern.solid(Color.kSpringGreen);
        this.m_white = LEDPattern.solid(Color.kFloralWhite);

        //NON-SOLID COLORS
        this.m_rainbow = LEDPattern.rainbow(0, 0);

        this.m_purpleGradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, 
        Color.kViolet, Color.kLavender);

        this.m_redGradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous,
        Color.kRed, Color.kCrimson);

        this.m_greenBase = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous,
        Color.kGreen, Color.kBlack);

        this.m_greenPattern = m_greenBase.blink(Units.Seconds.of(1));  

        this.m_sycned = m_greenBase.synchronizedBlink(RobotController::getRSLState);
    }  
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        m_led.setData(m_buffer);
    }

    public void red () {
        m_red.applyTo(m_buffer);
        m_led.setData(m_buffer);
    }
    
    public void violet () {
        m_red.applyTo(m_buffer);
        m_led.setData(m_buffer);
    }

    public void greenBlink(){
        m_greenPattern.applyTo(m_buffer);
        m_led.setData(m_buffer);
    }
}


