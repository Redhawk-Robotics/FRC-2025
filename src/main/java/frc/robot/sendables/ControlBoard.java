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
import frc.robot.Commands.PositionerFactory;
import frc.robot.subsystems.AlgaeFloorIntake;
import frc.robot.subsystems.AlgaeHandler;
import frc.robot.subsystems.CoralHandler;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;

public class ControlBoard implements Sendable {
    private Map<String, Supplier<Command>> states;
    private String currentState;
    private Command currentCmd;

    private final Elevator elevator;
    private final Pivot pivot;
    private final CoralHandler coral;
    private final AlgaeHandler algae;
    private final AlgaeFloorIntake spoiler;

    public ControlBoard(Elevator elevator, Pivot pivot, CoralHandler coral, AlgaeHandler algae,
            AlgaeFloorIntake spoiler) {
        this.states = new HashMap<>(Map.ofEntries(//
                Map.entry("Default", this::Default), //
                Map.entry("Coral.Contain", this::CoralContain), //
                Map.entry("Coral.Feed", this::CoralFeed), //
                Map.entry("Coral.L1", this::CoralL1), //
                Map.entry("Coral.L2", this::CoralL2), //
                Map.entry("Coral.L3", this::CoralL3), //
                Map.entry("Coral.L4", this::CoralL4), //
                Map.entry("Algae.L2", this::AlgaeL2), //
                Map.entry("Algae.L3", this::AlgaeL3), //
                // Map.entry("Algae.GroundIntake", this::AlgaeGroundIntake), //
                Map.entry("Algae.BargeFromGround", this::AlgaeBargeFromGround)));
        this.currentState = "";

        this.elevator = elevator;
        this.pivot = pivot;
        this.coral = coral;
        this.algae = algae;
        this.spoiler = spoiler;
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
        if (this.currentCmd != null && this.currentCmd.isScheduled()) {
            this.cancelCurrentCommand();
        }
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
        }).withName("ControlBoard.notFound(" + state + ")");
    }

    // TODO remove this
    private static Command notImplemented(String state) {
        return Commands.runOnce(() -> {
            DriverStation.reportError("State \"" + state + "\" not implemented!", false);
        }).withName("ControlBoard.notImplemented(" + state + ")");
    }

    private Command Default() {
        return PositionerFactory
                .Feed(this.elevator, this.pivot, this.coral, this.algae, this.spoiler)
                .withName("ControlBoard.Default");
    }

    private Command CoralFeed() {
        return PositionerFactory
                .Feed(this.elevator, this.pivot, this.coral, this.algae, this.spoiler)
                .withName("ControlBoard.CoralFeed");
    }

    private Command CoralContain() {
        return PositionerFactory
                .Attack(this.elevator, this.pivot, this.coral, this.algae, this.spoiler)
                .withName("ControlBoard.Attack(stow)");
    }

    private Command CoralL1() {
        return PositionerFactory.L1(this.elevator, this.pivot, this.coral, this.algae, this.spoiler)
                .withName("ControlBoard.CoralL1");
    }

    private Command CoralL2() {
        return PositionerFactory.L2(this.elevator, this.pivot, this.coral, this.algae, this.spoiler)
                .withName("ControlBoard.CoralL2");
    }

    private Command CoralL3() {
        return PositionerFactory.L3(this.elevator, this.pivot, this.coral, this.algae, this.spoiler)
                .withName("ControlBoard.CoralL3");
    }

    private Command CoralL4() {
        return PositionerFactory.L4(this.elevator, this.pivot, this.coral, this.algae, this.spoiler)
                .withName("ControlBoard.CoralL4");
    }

    private Command AlgaeL2() {
        return PositionerFactory.AlgaeL2(this.elevator, this.pivot, this.coral, this.algae, this.spoiler)
                .withName("ControlBoard.AlgaeL2");
    }

    private Command AlgaeL3() {
        return PositionerFactory.AlgaeL3(this.elevator, this.pivot, this.coral, this.algae, this.spoiler)
                .withName("ControlBoard.AlgaeL3");
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
        return PositionerFactory
                .Barge(this.elevator, this.pivot, this.coral, this.algae, this.spoiler)
                // .andThen(this.algae.rotateCCW_Outtake()).andThen(Commands.waitSeconds(.1))
                // .andThen(this.algae.stop())
                .withName("ControlBoard.Barge");
    }
}
