// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Settings;

public class CANRanges extends SubsystemBase {
    /** Creates a new coralAligner. */

    // I made a seperate subsystem for these things because I don't want to touch swerve
    private final CANrange m_leftCanRange;
    private final CANrange m_rightCanRange;

    private double m_leftDistance, m_rightDistance;

    public CANRanges() {
        m_leftCanRange =
                new CANrange(Settings.CoralAligner.CAN.kLEFT, Settings.CoralAligner.CAN.kBUS);
        m_rightCanRange =
                new CANrange(Settings.CoralAligner.CAN.kRIGHT, Settings.CoralAligner.CAN.kBUS);
        configure();
    }

    private void configure() {
        //null pointer exception thrown here
        
        CANrangeConfiguration config = new CANrangeConfiguration();
        //TODO What config settings do we need?
        config.ProximityParams.ProximityThreshold = 1;
        config.ProximityParams.MinSignalStrengthForValidMeasurement = 2500;
        config.ProximityParams.ProximityHysteresis = 0.1;
        m_leftCanRange.getConfigurator().apply(config);
        m_rightCanRange.getConfigurator().apply(config);

    }

    public void updateDistances() {
        m_leftDistance = m_leftCanRange.getDistance().getValueAsDouble();
        m_rightDistance = m_rightCanRange.getDistance().getValueAsDouble();
    }

    public boolean isAlignedLeft() {
        var isDetected = this.m_leftCanRange.getIsDetected(true);
        var value = isDetected.getValue();
        if (value == null) {
            DriverStation.reportWarning("[LEFT] isDetected gave null Boolean", false);
            return false;
        }
        return !value.booleanValue();
    }

    public boolean isAlignedRight() {
        var isDetected = this.m_rightCanRange.getIsDetected(true);
        var value = isDetected.getValue();
        if (value == null) {
            DriverStation.reportWarning("[RIGHT] isDetected gave null Boolean", false);
            return false;
        }
        return !value.booleanValue();
    }

    @Override
    public void periodic() {
        updateDistances();

        SmartDashboard.putNumber("CANRANGE/Left Distance", m_leftDistance);
        SmartDashboard.putNumber("CANRANGE/Right Distance", m_rightDistance);

        SmartDashboard.putBoolean("CANRANGE/Is Aligned Left", isAlignedLeft());
        SmartDashboard.putBoolean("CANRANGE/Is Aligned Right", isAlignedRight());

    }
}
