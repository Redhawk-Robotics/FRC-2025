// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sendables;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

public class ControlBoard implements Sendable {
    private Map<String, Supplier<Command>> states;
    private String currentState;
    private Command currentCmd;

    public ControlBoard() {
        this.states = new HashMap<>(Map.ofEntries(//
                Map.entry("State/Default", ControlBoard::Default), //
                Map.entry("State/Coral.Feed", ControlBoard::CoralFeed), //
                Map.entry("State/Coral.L1", ControlBoard::CoralL1), //
                Map.entry("State/Coral.L2", ControlBoard::CoralL2), //
                Map.entry("State/Coral.L3", ControlBoard::CoralL3), //
                Map.entry("State/Coral.L4", ControlBoard::CoralL4), //
                Map.entry("State/Algae.GroundIntake", Commands::none), //
                Map.entry("State/Algae.BargeFromGround", Commands::none)));
        this.currentState = "";
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // getter = SmartDashboard is requesting the current value
        // setter = SmartDashboard is giving the Robot a value


        for (String state : this.states.keySet()) {
            builder.addBooleanProperty(//
                    state, //
                    () -> this.currentState == state, //
                    val -> this.setState(state));
        }

        builder.addBooleanProperty("Deselect", () -> false, val -> this.reset());
        builder.addStringProperty("Status",
                () -> String
                        .format("State: [%s]\nStatus: [%s]", this.currentState,
                                this.currentCmd == null ? "null"
                                        : this.currentCmd.isScheduled() ? "running"
                                                : this.currentCmd.isFinished() ? "done" : "?"),
                null);
    }

    private void reset() {
        this.currentState = "";
        this.cancelCurrentCommand();
    }

    private void setState(String state) {
        this.currentState = state;
        this.currentCmd = this.states
                .getOrDefault(this.currentState, () -> ControlBoard.notFound(this.currentState))
                .get();
        this.scheduleCurrentCommand();
    }

    public void scheduleCurrentCommand() {
        if (!DriverStation.isTeleopEnabled()) {
            System.out.println("!!! Cannot run these commands outside of Teleop !!!");
            return;
        }
        // handles null commands
        CommandScheduler.getInstance().schedule(this.currentCmd);
    }

    public void cancelCurrentCommand() {
        // handles null commands
        CommandScheduler.getInstance().cancel(this.currentCmd);
    }

    private static Command notFound(String state) {
        return Commands.runOnce(() -> {
            DriverStation.reportWarning("State \"" + state + "\" not found!", false);
        });
    }

    // TODO remove this
    private static Command notImplemented(String state) {
        return Commands.runOnce(() -> {
            DriverStation.reportError("State \"" + state + "\" not implemented!", false);
        });
    }

    private static Command Default() {
        return notImplemented("Default()");
    }

    private static Command CoralFeed() {
        
        return notImplemented("CoralFeed()");
    }

    private static Command CoralL1() {
        return notImplemented("CoralL1()");
    }

    private static Command CoralL2() {
        return notImplemented("CoralL2()");
    }

    private static Command CoralL3() {
        return notImplemented("CoralL3()");
    }

    private static Command CoralL4() {
        return notImplemented("CoralL4()");
    }
}
