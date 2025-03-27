// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Settings;
import frc.robot.Constants.*;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;




public class LEDs extends SubsystemBase {
    private final CANdle m_candle;
    private final int m_LEDCount;
    private final RainbowAnimation rainbowAnim;
    private Animation m_toAnimate;
    private CANdleConfiguration m_configAll;
    

    public enum LEDColors {
        RED(255,0,4),
        YELLOW(255, 227, 28),
        GREEN(0, 218, 30),
        PINK(255, 28, 159),
        PURPLE(190, 69, 255);

        public final int red;
        public final int green;
        public final int blue;
        
        private LEDColors(int red, int green, int blue){
            this.red = red;
            this.green = green;
            this.blue = blue;
        }

    }

    /** Creates a new LEDs. */
    public LEDs() {
        this.m_candle = new CANdle(0);
        this.m_LEDCount = Settings.LEDs.LED_COUNT;
        this.rainbowAnim = new RainbowAnimation(1, 0.5, 64);
        this.m_configAll = new CANdleConfiguration();
        this.m_configAll = new CANdleConfiguration();
        this.m_configAll.statusLedOffWhenActive = true;
        this.m_configAll.disableWhenLOS = false;
        this.m_configAll.stripType = LEDStripType.RGB;
        this.m_configAll.vBatOutputMode = VBatOutputMode.Modulated;
        this.m_candle.configAllSettings(m_configAll, 100);
    }



    //Wrapers: allows us to access LEDS from subsystems
    public double getVbat() { return m_candle.getBusVoltage(); }
    public double get5V() { return m_candle.get5VRailVoltage(); }
    public double getCurrent() { return m_candle.getCurrent(); }
    public double getTemperature() { return m_candle.getTemperature(); }
    public void configBrightness(double percent) { m_candle.configBrightnessScalar(percent, 0); }
    public void configLos(boolean disableWhenLos) { m_candle.configLOSBehavior(disableWhenLos, 0); }
    public void configLedType(LEDStripType type) { m_candle.configLEDType(type, 0); }
    public void configStatusLedBehavior(boolean offWhenActive) { m_candle.configStatusLedState(offWhenActive, 0); }

    public void AnimationState() {
    }
    @Override
    public void periodic() {

        // This method will be called once per scheduler run
    }
}
