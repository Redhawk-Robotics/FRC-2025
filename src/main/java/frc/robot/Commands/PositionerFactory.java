// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.Optional;
import java.util.Stack;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.AlgaeFloorIntake;
import frc.robot.subsystems.AlgaeHandler;
import frc.robot.subsystems.CoralHandler;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;

public final class PositionerFactory {

    private static final class Settings {
        // conflicts:
        // - if elevator is below [threshold_e_s] (near the bottom), then the spoiler and pivot can collide
        //     when spoiler is below [threshold_s_p] and pivot is below [threshold_p_s]
        // - pivot cannot be below [threshold_p_e] if elevator is, or moves, between [threshold_e_p1, threshold_e_p2]
        // - elevator must be in range [min_e, max_e]
        // - pivot must be in range [min_p, max_p]
        // - spoiler must be in range [min_s, max_s]

        public static final double minPositionWhereElevatorFreesPivotAndSpoiler = 1;
        public static final double minPositionWherePivotDoesNotCollideWithSpoiler = 10;
        public static final double minPositionWhereSpoilerDoesNotCollideWithPivot = 1;
        public static final double minPositionWherePivotDoesNotCollideWithElevator = 0.38; // threshold_p_e (pivot blocks the elevator below this reference)
        public static final double minPositionWhereElevatorDoesNotBlockPivot = 4.5; // threshold_e_p1 (elevator blocks the pivot above this reference)
        public static final double maxPositionWhereElevatorDoesNotBlockPivot = 80; // threshold_e_p2 (elevator blocks the pivot below this reference)
        public static final double midpointPositionWhereElevatorBlocksPivot =
                (minPositionWhereElevatorDoesNotBlockPivot
                        + maxPositionWhereElevatorDoesNotBlockPivot) / 2; // (4.5+80)/2 = 42.25

        public static final double minElevatorPosition = 0;
        public static final double maxElevatorPosition = 108.6;
        public static final double allowedElevatorError = 0.25;

        public static final double minPivotPosition = 0.01;
        public static final double maxPivotPosition = 0.62;
        public static final double allowedPivotError = 0.005;

        public static final double minSpoilerPosition = 0;
        public static final double maxSpoilerPosition = 10;
        public static final double allowedSpoilerError = 0.5;

        public static final double ELEVATOR_FEED_POSITION = 0;
        public static final double ELEVATOR_ATTACK_POSITION = ELEVATOR_FEED_POSITION;
        public static final double ELEVATOR_L1_POSITION = ELEVATOR_FEED_POSITION;
        public static final double ELEVATOR_L2_POSITION = ELEVATOR_FEED_POSITION;
        public static final double ELEVATOR_L3_POSITION = 26.1;
        public static final double ELEVATOR_L4_POSITION = 108.5; //106.5
        public static final double ELEVATOR_BARGE_POSITION = ELEVATOR_L4_POSITION;
        public static final double ELEVATOR_ALGAE_L2_POSITION = ELEVATOR_FEED_POSITION;
        public static final double ELEVATOR_ALGAE_L3_POSITION = 57.2;

        public static final double PIVOT_FEED_POSITION = 0.055;
        public static final double PIVOT_ATTACK_POSITION = 0.600;
        public static final double PIVOT_L1_POSITION = PIVOT_FEED_POSITION;
        public static final double PIVOT_L2_POSITION = 0.56;
        public static final double PIVOT_L3_POSITION = 0.61;
        public static final double PIVOT_L4_POSITION = PIVOT_L3_POSITION;
        public static final double PIVOT_BARGE_POSITION = 0.510;
        public static final double PIVOT_ALGAE_L2_POSITION = 0.320;
        public static final double PIVOT_ALGAE_L3_POSITION = 0.28;

        public static final boolean Verbose = true;
    }

    static class State {
        // null means that component in the State is unchanged
        // between transitions
        Double elevatorSetPoint = null, //
                pivotSetPoint = null, //
                coralHandlerSpeed = null, //
                algaeHandlerSpeed = null, //
                spoilerSetPoint = null, //
                spoilerRollerSpeed = null;
        BooleanSupplier finished = () -> true;

        State() {}

        State(Double elevatorSetPoint, Double pivotSetPoint, Double coralHandlerSpeed,
                Double algaeHandlerSpeed, Double spoilerSetPoint, Double spoilerRollerSpeed,
                BooleanSupplier done) {
            this.elevatorSetPoint = elevatorSetPoint;
            this.pivotSetPoint = pivotSetPoint;
            this.coralHandlerSpeed = coralHandlerSpeed;
            this.algaeHandlerSpeed = algaeHandlerSpeed;
            this.spoilerSetPoint = spoilerSetPoint;
            this.spoilerRollerSpeed = spoilerRollerSpeed;
            if (done != null) {
                this.finished = done;
            }
        }

        public State merge(State other) {
            return new State(
                    this.elevatorSetPoint != null ? this.elevatorSetPoint : other.elevatorSetPoint,
                    this.pivotSetPoint != null ? this.pivotSetPoint : other.pivotSetPoint,
                    this.coralHandlerSpeed != null ? this.coralHandlerSpeed
                            : other.coralHandlerSpeed,
                    this.algaeHandlerSpeed != null ? this.algaeHandlerSpeed
                            : other.algaeHandlerSpeed,
                    this.spoilerSetPoint != null ? this.spoilerSetPoint : other.spoilerSetPoint,
                    this.spoilerRollerSpeed != null ? this.spoilerRollerSpeed
                            : other.spoilerRollerSpeed,
                    () -> {
                        return this.finished.getAsBoolean() && other.finished.getAsBoolean();
                    });
        }

        //todo look over this, may be causing slow performance
        public String toString() {
            if (Settings.Verbose) {
                return String.format("State(e:%s, p:%s, c:%s, a:%s, s:%s, r:%s)",
                        this.elevatorSetPoint == null ? "null" : this.elevatorSetPoint,
                        this.pivotSetPoint == null ? "null" : this.pivotSetPoint,
                        this.coralHandlerSpeed == null ? "null" : this.coralHandlerSpeed,
                        this.algaeHandlerSpeed == null ? "null" : this.algaeHandlerSpeed,
                        this.spoilerSetPoint == null ? "null" : this.spoilerSetPoint,
                        this.spoilerRollerSpeed == null ? "null" : this.spoilerRollerSpeed);
            }
            return "State(...)";
        }
    }

    static class GoToState extends Command {
        private State goal;
        private Stack<State> trajectory;
        private Elevator elevator;
        private Pivot pivot;
        private CoralHandler coral;
        private AlgaeHandler algae;
        private AlgaeFloorIntake spoiler;

        GoToState(State goal, Elevator elevator, Pivot pivot, CoralHandler coral,
                AlgaeHandler algae, AlgaeFloorIntake spoiler) {
            this.trajectory = new Stack<>();
            if (goal == null) {
                DriverStation.reportWarning("[GoToState] received null goal State %s", true);
                goal = new State();
            }
            this.goal = goal;
            this.elevator = elevator;
            this.pivot = pivot;
            this.coral = coral;
            this.algae = algae;
            this.spoiler = spoiler;
            this.addRequirements(elevator, pivot, coral, algae, spoiler);
        }

        private static int zone(State s) {
            // returns the "zone" that this State is in
            // returns 0 if oneOf(pivot,elevator) setpoints are null
            if (s.pivotSetPoint == null || s.elevatorSetPoint == null) {
                return 0;
            }
            // else returns oneOf(1,2,3,4)
            if (s.pivotSetPoint < Settings.minPositionWherePivotDoesNotCollideWithElevator) {
                // Z1 or Z4
                if (s.elevatorSetPoint < Settings.midpointPositionWhereElevatorBlocksPivot) {
                    // Z1
                    return 1;
                } else {
                    // Z4
                    return 4;
                }
            } else {
                // Z2 or Z3
                if (s.elevatorSetPoint < Settings.midpointPositionWhereElevatorBlocksPivot) {
                    // Z2
                    return 2;
                } else {
                    // Z3
                    return 3;
                }
            }
        }

        // TODO this should also take in the spoiler for spoiler+pivot conflicts
        static Optional<State[]> getMidList(State start, State end, Elevator elevator,
                Pivot pivot) {
            State[] result;
            // Add states to remove conflicts
            //
            // How?
            //
            // We split into 4 "zones" (see `./positioning-zones.png`)
            // Rule: the robot can only move between numerically adjacent zones
            // So, if you are in Z1 and need to get to Z4, then
            // you have to go to Z2, then to Z3, _then_ to Z4.
            //
            // In practice, you might want to go from Feed (Z1) to L4 (Z3)
            // so you first go to a safe point in Z2 = (goal.pivot, elevator.minBlocking)
            // and _then_ to L4 = (goal.pivot, goal.elevator).
            //
            // You can think of the "zones" as a sort-of "XY" coordinate system
            // where X == the pivot's horizontal component
            // and Y == the elevator's vertical component.
            //
            // "zones" are highly dependent on the three pivot/elevator
            // blocking parameters.

            int startZone = zone(start);
            int endZone = zone(end);
            if (startZone == 0 || endZone == 0) {
                DriverStation.reportError(String.format(
                        "Could not determine State zones for %s->%s (got zones %d->%d)",
                        start.toString(), end.toString(), startZone, endZone), false);
                // make this command a no-op
                return Optional.empty();
            }
            // examples:
            // (4-1)-1 = 2 (zone 4->1, need 2 extra zones (z3 and z2))
            // (3-1)-1 = 1 (zone 3->1, need 1 extra zone (z2))
            // (2-1)-1 = 0 (zone 2->1, can go direct b/c they're adjacent)
            // (1-1)-1 = -1 (same zone, can go direct)
            int extraStates = Math.abs(startZone - endZone) - 1;
            if (extraStates > 2) {
                DriverStation
                        .reportError(
                                String.format("Invalid zones for States %s->%s (got zones %d->%d)",
                                        start.toString(), end.toString(), startZone, endZone),
                                false);
                // make this command a no-op
                return Optional.empty();
            } else if (extraStates == 1) {
                // curr -> mid -> goal
                // Z1->Z2->Z3 (or reverse) (Feed -> L4)
                // Z2->Z3->Z4 (or reverse)
                // So, we need to know if we're going to Z2 or Z3
                double pivotPos;
                double elevatorPos;
                if (startZone < endZone) {
                    if (startZone == 1) {
                        // Z1 -> Z3
                        // Z2 intermediate (goal.pivot, elevator.minBlocking)
                        pivotPos = end.pivotSetPoint;
                        elevatorPos = Settings.minPositionWhereElevatorDoesNotBlockPivot;
                    } else {
                        // Z2 -> Z4
                        // Z3 intermediate (pivot.minBlocking, goal.elevator)
                        pivotPos = Settings.minPositionWherePivotDoesNotCollideWithElevator;
                        elevatorPos = end.elevatorSetPoint;
                    }
                } else {
                    if (startZone == 4) {
                        // Z4 -> Z2
                        // Z3 intermediate (goal.pivot, elevator.maxBlocking)
                        pivotPos = end.pivotSetPoint;
                        elevatorPos = Settings.maxPositionWhereElevatorDoesNotBlockPivot;
                    } else {
                        // Z3 -> Z1
                        // Z2 intermediate (pivot.minBlocking, goal.elevator)
                        pivotPos = Settings.minPositionWherePivotDoesNotCollideWithElevator;
                        elevatorPos = end.elevatorSetPoint;
                    }
                }
                State mid = PositionerFactory.ElevatorAndPivotToPosition(elevator, elevatorPos,
                        pivot, pivotPos);
                result = new State[] {mid};
            } else if (extraStates == 2) {
                // Z1->Z2->Z3->Z4 (or reverse)

                // Z1->Z2 or Z3->Z2
                // (pivot.minBlocking, elevator.minBlocking)
                State firstMid = PositionerFactory.ElevatorAndPivotToPosition(elevator,
                        Settings.minPositionWhereElevatorDoesNotBlockPivot, pivot,
                        Settings.minPositionWherePivotDoesNotCollideWithElevator);

                // Z2->Z3 or Z4->Z3
                // (pivot.minBlocking, elevator.maxBlocking)
                State secondMid = PositionerFactory.ElevatorAndPivotToPosition(elevator,
                        Settings.maxPositionWhereElevatorDoesNotBlockPivot, pivot,
                        Settings.minPositionWherePivotDoesNotCollideWithElevator);

                if (startZone < endZone) {
                    // Z1->Z2->Z3->Z4
                    // nothing to do:
                    // - firstMid = Z1->Z2
                    // - secondMid = Z2->Z3
                    result = new State[] {firstMid, secondMid};
                } else {
                    // Z4->Z3->Z2->Z1
                    // need to swap firstMid and secondMid
                    result = new State[] {secondMid, firstMid};
                }
            } else {
                // no extra states
                // either we're going within the same zone
                // or we're going to an adjacent zone
                result = new State[] {}; // give it an empty list, add no middle states
            }
            return Optional.of(result);
        }

        private State currentGoalState() {
            if (this.isFinished()) {
                return new State(); // todo does this fix it?
            }
            return this.trajectory.peek();
        }

        private boolean isCurrentStateFinished() {
            return this.currentGoalState().finished.getAsBoolean();
        }

        private void proceedToNextState() {
            State next = this.trajectory.pop();
            if (Settings.Verbose) {
                System.out.printf("[GoToState] finished state: %s\n", next.toString());
            }
            this.runCurrentState();
        }

        private void runCurrentState() {
            if (this.isFinished()) {
                return;
            }
            State curr = this.currentGoalState();
            // now actually go to the state
            if (curr.elevatorSetPoint != null) {
                this.elevator.setReference(curr.elevatorSetPoint);
            }
            if (curr.pivotSetPoint != null) {
                this.pivot.setReference(curr.pivotSetPoint);
            }
            if (curr.coralHandlerSpeed != null) {
                this.coral.setSpeed(curr.coralHandlerSpeed);
            }
            // this one handles passed-in nulls
            this.spoiler.setArmAndRoller(curr.spoilerSetPoint, curr.spoilerRollerSpeed);
        }

        @Override
        public void initialize() {
            // Get the goal state
            State goal = this.goal;
            this.trajectory.push(this.goal); // add the goal state to the trajectory

            if (goal.elevatorSetPoint == null && //
                    goal.pivotSetPoint == null && //
                    goal.spoilerSetPoint == null //
            ) {
                System.out.println("[GoToState] request to re-position nothing");
                // No conflicts to resolve
                this.runCurrentState();
                return;
            }

            if (goal.elevatorSetPoint != null
                    && (goal.elevatorSetPoint < Settings.minElevatorPosition
                            || goal.elevatorSetPoint > Settings.maxElevatorPosition)) {
                DriverStation.reportError("[GoToState] cannot put Elevator out of bounds", true);
                // make this command a no-op
                this.trajectory.clear();
                this.trajectory.push(new State());
                this.runCurrentState();
                return;
            }
            if (goal.pivotSetPoint != null && (goal.pivotSetPoint < Settings.minPivotPosition
                    || goal.pivotSetPoint > Settings.maxPivotPosition)) {
                DriverStation.reportError("[GoToState] cannot put Pivot out of bounds", true);
                // make this command a no-op
                this.trajectory.clear();
                this.trajectory.push(new State());
                this.runCurrentState();
                return;
            }
            if (goal.spoilerSetPoint != null && (goal.spoilerSetPoint < Settings.minSpoilerPosition
                    || goal.spoilerSetPoint > Settings.maxSpoilerPosition)) {
                DriverStation.reportError("[GoToState] cannot put Spoiler out of bounds", true);
                // make this command a no-op
                this.trajectory.clear();
                this.trajectory.push(new State());
                this.runCurrentState();
                return;
            }

            // Get current state
            State curr = new State(this.elevator.getPosition(), this.pivot.getPosition(),
                    this.coral.getSpeed(), this.algae.getSpeed(), this.spoiler.getArmPosition(),
                    this.spoiler.getRollerSpeed(), null); // done == null b/c we're already finished
            State fullGoal = goal.merge(curr);

            Optional<State[]> maybeResult =
                    GoToState.getMidList(curr, fullGoal, this.elevator, this.pivot);
            if (maybeResult.isEmpty()) {
                // make this command a no-op
                this.trajectory.clear();
                this.trajectory.push(new State());
                this.runCurrentState();
                return;
            }
            State[] list = maybeResult.get();
            for (int i = list.length - 1; i >= 0; i--) {
                this.trajectory.push(list[i]);
            }
            if (Settings.Verbose) {
                System.out.printf("[GoToState] generated trajectory:\n%s\n", this.trajectory);
            }
            this.runCurrentState();
        }

        @Override
        public void execute() {
            if (this.isCurrentStateFinished()) {
                this.proceedToNextState();
            }
        }

        @Override
        public void end(boolean interrupted) {
            if (Settings.Verbose) {
                System.out.printf("[GoToState] ended (interrupted:%b)\n", interrupted);
            }
            if (!interrupted) {
                return;
            }
            this.trajectory.clear();
            // stop all subsystems at the current state
            State curr = this.currentGoalState();
            if (curr.elevatorSetPoint != null) {
                this.elevator.stopElevator();
            }
            if (curr.pivotSetPoint != null) {
                this.pivot.stopPivot();
            }
            if (curr.spoilerSetPoint != null) {
                this.spoiler.stopArm();
            }
        }

        @Override
        public boolean isFinished() {
            return this.trajectory.empty();
        }
    }

    // helper function
    private static State ElevatorAndPivotToPosition(Elevator elevator, double elevatorPosition,
            Pivot pivot, double pivotPosition) {
        BooleanSupplier done = () -> {
            return (Math
                    .abs(elevator.getPosition() - elevatorPosition) < Settings.allowedElevatorError)
                    && (Math.abs(pivot.getPosition() - pivotPosition) < Settings.allowedPivotError);
        };
        return new State(elevatorPosition, pivotPosition, null, null, null, null, done);
    }

    public static Command Feed(Elevator elevator, Pivot pivot, CoralHandler coral,
            AlgaeHandler algae, AlgaeFloorIntake spoiler) {

        // also default position w/o Coral
        State goal = PositionerFactory.ElevatorAndPivotToPosition(elevator,
                Settings.ELEVATOR_FEED_POSITION, pivot, Settings.PIVOT_FEED_POSITION);
        return new GoToState(goal, elevator, pivot, coral, algae, spoiler).withName("Feed");
    }

    public static Command Attack(Elevator elevator, Pivot pivot, CoralHandler coral,
            AlgaeHandler algae, AlgaeFloorIntake spoiler) {

        State goal = PositionerFactory.ElevatorAndPivotToPosition(elevator,
                Settings.ELEVATOR_ATTACK_POSITION, pivot, Settings.PIVOT_ATTACK_POSITION);
        return new GoToState(goal, elevator, pivot, coral, algae, spoiler).withName("Attack");
    }

    public static Command L1(Elevator elevator, Pivot pivot, CoralHandler coral, AlgaeHandler algae,
            AlgaeFloorIntake spoiler) {

        State goal = PositionerFactory.ElevatorAndPivotToPosition(elevator,
                Settings.ELEVATOR_L1_POSITION, pivot, Settings.PIVOT_L1_POSITION);
        return new GoToState(goal, elevator, pivot, coral, algae, spoiler).withName("L1");
    }

    public static Command L2(Elevator elevator, Pivot pivot, CoralHandler coral, AlgaeHandler algae,
            AlgaeFloorIntake spoiler) {

        State goal = PositionerFactory.ElevatorAndPivotToPosition(elevator,
                Settings.ELEVATOR_L2_POSITION, pivot, Settings.PIVOT_L2_POSITION);
        return new GoToState(goal, elevator, pivot, coral, algae, spoiler).withName("L2");
    }

    public static Command L3(Elevator elevator, Pivot pivot, CoralHandler coral, AlgaeHandler algae,
            AlgaeFloorIntake spoiler) {

        State goal = PositionerFactory.ElevatorAndPivotToPosition(elevator,
                Settings.ELEVATOR_L3_POSITION, pivot, Settings.PIVOT_L3_POSITION);
        return new GoToState(goal, elevator, pivot, coral, algae, spoiler).withName("L3");
    }

    public static Command L4(Elevator elevator, Pivot pivot, CoralHandler coral, AlgaeHandler algae,
            AlgaeFloorIntake spoiler) {

        State goal = PositionerFactory.ElevatorAndPivotToPosition(elevator,
                Settings.ELEVATOR_L4_POSITION, pivot, Settings.PIVOT_L4_POSITION);
        return new GoToState(goal, elevator, pivot, coral, algae, spoiler).withName("L4");
    }

    public static Command Barge(Elevator elevator, Pivot pivot, CoralHandler coral,
            AlgaeHandler algae, AlgaeFloorIntake spoiler) {

        State goal = PositionerFactory.ElevatorAndPivotToPosition(elevator,
                Settings.ELEVATOR_BARGE_POSITION, pivot, Settings.PIVOT_BARGE_POSITION);
        return new GoToState(goal, elevator, pivot, coral, algae, spoiler).withName("Barge");
    }

    public static Command AlgaeL2(Elevator elevator, Pivot pivot, CoralHandler coral,
            AlgaeHandler algae, AlgaeFloorIntake spoiler) {

        State goal = PositionerFactory.ElevatorAndPivotToPosition(elevator,
                Settings.ELEVATOR_ALGAE_L2_POSITION, pivot, Settings.PIVOT_ALGAE_L2_POSITION);
        return new GoToState(goal, elevator, pivot, coral, algae, spoiler).withName("Algae.L2");
    }

    public static Command AlgaeL3(Elevator elevator, Pivot pivot, CoralHandler coral,
            AlgaeHandler algae, AlgaeFloorIntake spoiler) {

        State goal = PositionerFactory.ElevatorAndPivotToPosition(elevator,
                Settings.ELEVATOR_ALGAE_L3_POSITION, pivot, Settings.PIVOT_ALGAE_L3_POSITION);
        return new GoToState(goal, elevator, pivot, coral, algae, spoiler).withName("Algae.L3");
    }

    public static Command Stop(Elevator elevator, Pivot pivot, CoralHandler coral,
            AlgaeHandler algae, AlgaeFloorIntake spoiler) {
        Command result = Commands.parallel(//
                elevator.runOnce(() -> elevator.stopElevator()),
                pivot.runOnce(() -> pivot.stopPivot()), coral.runOnce(() -> coral.setSpeed(0)),
                algae.runOnce(() -> algae.setSpeed(0)),
                spoiler.runOnce(() -> spoiler.stopArmAndRoller()));
        result.addRequirements(pivot, elevator, coral, algae, spoiler);
        return result;
    }
}
