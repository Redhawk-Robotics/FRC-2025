// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.databind.jsontype.impl.AsDeductionTypeDeserializer;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// TODO -- set this up
// https://docs.revrobotics.com/rev-crossover-products/blinkin/gs

public class LEDs extends SubsystemBase {
    AddressableLED m_led1;
    AddressableLED m_led2;
    AddressableLED m_led3;

    AddressableLEDBuffer m_ledBuffer1;
    AddressableLEDBuffer m_ledBuffer2;
    AddressableLEDBuffer m_ledBuffer3;

    /** Creates a new LEDs Subsystem. */

    public LEDs() {
        //TODO Find these
        AddressableLED m_led1 = new AddressableLED(0);
        AddressableLED m_led2 = new AddressableLED(0);
        AddressableLED m_led3 = new AddressableLED(0);

        // Figure out what this does lmao
        m_ledBuffer1 = new AddressableLEDBuffer(60);
        m_ledBuffer2 = new AddressableLEDBuffer(60);
        m_ledBuffer3 = new AddressableLEDBuffer(60);

        m_led1 = new AddressableLED(0);
        m_led2 = new AddressableLED(0);
        m_led3 = new AddressableLED(0);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
