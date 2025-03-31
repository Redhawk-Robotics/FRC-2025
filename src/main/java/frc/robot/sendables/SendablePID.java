// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sendables;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SendablePID implements Sendable {
    public static final String prefix  = "PID-Tuner";

    private String name;
    private float kP;
    private float kI;
    private float kD;
    private float setpoint;

    public SendablePID(String name, float init_P, float init_I, float init_D, float init_setPoint) {
        this.name = name;
        this.kP = init_P;
        this.kI = init_I;
        this.kD = init_D;
        this.setpoint = init_setPoint;
    }

    public SendablePID(String name) {
        this(name, 0f, 0f, 0f, 0f);
    }

    private static float clamp(float val, float low, float high) {
        if (val < low)
            return low;
        if (val > high)
            return high;
        return val;
    }

    public float P() {
        return this.kP;
    }

    public float I() {
        return this.kI;
    }

    public float D() {
        return this.kD;
    }

    public float SetPoint() {
        return this.setpoint;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // setter = SmartDashboard is giving the Robot a value
        // getter = SmartDashboard is requesting the current value

        builder.addFloatProperty("PID-Tuner/" + this.name + "/P", () -> this.kP,
                val -> this.kP = SendablePID.clamp(val, 0, 1000));
        builder.addFloatProperty("PID-Tuner/" + this.name + "/I", () -> this.kI,
                val -> this.kI = SendablePID.clamp(val, 0, 1000));
        builder.addFloatProperty("PID-Tuner/" + this.name + "/D", () -> this.kD,
                val -> this.kD = SendablePID.clamp(val, 0, 1000));
        builder.addFloatProperty("PID-Tuner/" + this.name + "/set-point", () -> this.setpoint,
                val -> this.setpoint = val);
    }
}
