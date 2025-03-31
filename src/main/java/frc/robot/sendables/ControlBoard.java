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
import frc.robot.Commands.CoralPositionFactory;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;

public class ControlBoard implements Sendable {
    private Map<String, Supplier<Command>> states;
    private String currentState;
    private Command currentCmd;

    private final Elevator elevator;
    private final Pivot pivot;

    public ControlBoard(Elevator elevator, Pivot pivot) {
        this.states = new HashMap<>(Map.ofEntries(//
                Map.entry("Default", this::Default), //
                Map.entry("Coral.Feed", this::CoralFeed), //
                Map.entry("Coral.L1", this::CoralL1), //
                Map.entry("Coral.L2", this::CoralL2), //
                Map.entry("Coral.L3", this::CoralL3), //
                Map.entry("Coral.L4", this::CoralL4), //
                Map.entry("Algae.GroundIntake", this::AlgaeGroundIntake), //
                Map.entry("Algae.BargeFromGround", this::AlgaeBargeFromGround)));
        this.currentState = "";

        this.elevator = elevator;
        this.pivot = pivot;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // getter = SmartDashboard is requesting the current value
        // setter = SmartDashboard is giving the Robot a value

        builder.addBooleanProperty("Deselect", () -> false, val -> this.reset());
        builder.addStringProperty("Status",
                () -> String
                        .format("\"%s\" is [%s]", this.currentState,
                                this.currentCmd == null ? "null"
                                        : this.currentCmd.isScheduled() ? "running"
                                                : this.currentCmd.isFinished() ? "done" : "?"),
                null);

        for (String state : this.states.keySet()) {
            builder.addBooleanProperty(//
                    state, //
                    () -> this.currentState == state, //
                    val -> this.setState(state));
        }
    }

    private void reset() {
        this.currentState = "";
        this.cancelCurrentCommand();
        this.currentCmd = null;
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
        }).withName("ControlBoard.notFound("+state+")");
    }

    // TODO remove this
    private static Command notImplemented(String state) {
        return Commands.runOnce(() -> {
            DriverStation.reportError("State \"" + state + "\" not implemented!", false);
        }).withName("ControlBoard.notImplemented("+state+")");
    }

    private Command Default() {
        return CoralPositionFactory.Feed(this.elevator, this.pivot).withName("ControlBoard.Default");
    }

    private Command CoralFeed() {
        return CoralPositionFactory.Feed(this.elevator, this.pivot).withName("ControlBoard.CoralFeed");
    }

    private Command CoralL1() {
        return CoralPositionFactory.L1(this.elevator, this.pivot).withName("ControlBoard.CoralL1");
    }

    private Command CoralL2() {
        return ControlBoard.notImplemented("CoralL2()");
    }

    private Command CoralL3() {
        return ControlBoard.notImplemented("CoralL3()");
    }

    private Command CoralL4() {
        return ControlBoard.notImplemented("CoralL4()");
    }

    private Command AlgaeGroundIntake() {
        // - move spoiler out
        // - turn on roller
        // - move pivot to intake
        // - turn on algae handler wheel
        // - when the algae handler wheel outputCurrent spikes, then set the algae handler wheel to a low intake value
        return ControlBoard.notImplemented("AlgaeGroundIntake()");
    }

    private Command AlgaeBargeFromGround() {
        // - rotate pivot up
        // - when pivot is safe, move elevator all the way up
        // - when both are there, reverse the algae handler wheel
        // - wait 100ms
        // - move elevator and pivot back down ?
        return ControlBoard.notImplemented("AlgaeBargeFromGround()");
    }
}
