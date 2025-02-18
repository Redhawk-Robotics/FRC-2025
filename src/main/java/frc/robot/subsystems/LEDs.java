// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// TODO -- set this up
// https://docs.revrobotics.com/rev-crossover-products/blinkin/gs

public class LEDs extends SubsystemBase {

    public enum COLORS {
        kHOTPINK,
        kCONFETTI,
        kVIOLET
    }

    private final Spark m_LEDS;
    private double led_color;

    public LEDs() {
        m_LEDS = new Spark(9);
        led_color = 0.57;
    }

    public void setColorValue(COLORS color) {
        switch (color) {
            case kCONFETTI:
                led_color = -0.87;
                break;
            case kVIOLET:
                led_color = 0.91;
                break;
            case kHOTPINK:
                led_color = 0.57;

        }
        //hot pink 0.57
        //confetti -.87
        //violet PRETTY .91
    }

    //TODO TEST THIS MAKE SURE IT DOESNT EXPLODE THE RIO AND DOESNT LAG IT
    public Command setToViolet() {
        return this.runOnce( () -> {setColorValue(COLORS.kVIOLET);});
    }

    public Command setToConfetti() {
        return this.runOnce( () -> {setColorValue(COLORS.kCONFETTI);} );
    }

    public Command setToHotPink() {
        return this.runOnce( () -> {setColorValue(COLORS.kHOTPINK);} );
    }



    @Override
    public void periodic() {
       m_LEDS.set(led_color);
    }
}
