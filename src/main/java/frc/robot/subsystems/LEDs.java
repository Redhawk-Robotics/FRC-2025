// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// TODO -- set this up
// https://docs.revrobotics.com/rev-crossover-products/blinkin/gs

public class LEDs extends SubsystemBase {
    /** Creates a new LEDs Subsystem. */
    AddressableLED m_Led = new AddressableLED(9);
    public LEDs() {}

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
