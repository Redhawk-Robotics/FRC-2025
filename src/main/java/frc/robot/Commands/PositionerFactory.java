// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.ArrayList;
import java.util.List;
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
        public static final double minPositionWherePivotDoesNotCollideWithElevator = 120; // threshold_p_e (pivot blocks the elevator below this reference)
        public static final double minPositionWhereElevatorDoesNotBlockPivot = 5; // threshold_e_p1 (elevator blocks the pivot above this reference)
        public static final double maxPositionWhereElevatorDoesNotBlockPivot = 55; // threshold_e_p2 (elevator blocks the pivot below this reference)
        public static final double midpointPositionWhereElevatorBlocksPivot =
                (minPositionWhereElevatorDoesNotBlockPivot
                        + maxPositionWhereElevatorDoesNotBlockPivot) / 2;

        public static final double minElevatorPosition = 0;
        public static final double maxElevatorPosition = 107;
        public static final double allowedElevatorError = 0.5;

        public static final double minPivotPosition = 70;
        public static final double maxPivotPosition = 181;
        public static final double allowedPivotError = 0.5;

        public static final double minSpoilerPosition = 0;
        public static final double maxSpoilerPosition = 10;
        public static final double allowedSpoilerError = 0.5;

        public static final double ELEVATOR_FEED_POSITION = 0;
        public static final double ELEVATOR_L1_POSITION = ELEVATOR_FEED_POSITION;
        public static final double ELEVATOR_L2_POSITION = ELEVATOR_FEED_POSITION;
        public static final double ELEVATOR_L3_POSITION = ELEVATOR_FEED_POSITION;
        public static final double ELEVATOR_L4_POSITION = 106;

        public static final double PIVOT_FEED_POSITION = 72;
        public static final double PIVOT_L1_POSITION = 145;
        public static final double PIVOT_L2_POSITION = 153;
        public static final double PIVOT_L3_POSITION = 180;
        public static final double PIVOT_L4_POSITION = 166;
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

        public String toString() {
            return String.format("State(e:%s, p:%s, c:%s, a:%s, s:%s, r:%s)",
                    this.elevatorSetPoint == null ? "null" : this.elevatorSetPoint,
                    this.pivotSetPoint == null ? "null" : this.pivotSetPoint,
                    this.coralHandlerSpeed == null ? "null" : this.coralHandlerSpeed,
                    this.algaeHandlerSpeed == null ? "null" : this.algaeHandlerSpeed,
                    this.spoilerSetPoint == null ? "null" : this.spoilerSetPoint,
                    this.spoilerRollerSpeed == null ? "null" : this.spoilerRollerSpeed);
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

        static Optional<List<State>> getMidList(State start, State end, Elevator elevator, Pivot pivot) {
            ArrayList<State> result = new ArrayList<>();

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
            int extraStates = Math.abs(startZone - endZone) - 1;
            if (extraStates > 2) {
                DriverStation
                        .reportError(
                                String.format("Invalid zones for States %s->%s (got zones %d->%d)",
                                        start.toString(), end.toString(), startZone, endZone),
                                false);
                // make this command a no-op
                return Optional.empty();
            }
            if (extraStates == 1) {
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
                BooleanSupplier done = () -> {
                    return (Math
                            .abs(pivot.getPosition() - pivotPos) < Settings.allowedPivotError)
                            && (Math.abs(elevator.getPosition()
                                    - elevatorPos) < Settings.allowedElevatorError);
                };
                State mid = new State(elevatorPos, pivotPos, null, null, end.spoilerSetPoint, null,
                        done);
                result.add(mid);
            } else {
                // Z1->Z2->Z3->Z4 (or reverse)

                // Z1->Z2 or Z3->Z2
                // (pivot.minBlocking, elevator.minBlocking)
                State firstMid = new State(Settings.minPositionWhereElevatorDoesNotBlockPivot,
                        Settings.minPositionWherePivotDoesNotCollideWithElevator, null, null,
                        end.spoilerSetPoint, null, () -> {
                            return (Math.abs(pivot.getPosition()
                                    - Settings.minPositionWherePivotDoesNotCollideWithElevator) < Settings.allowedPivotError)
                                    && (Math.abs(elevator.getPosition()
                                            - Settings.minPositionWhereElevatorDoesNotBlockPivot) < Settings.allowedElevatorError);
                        });

                // Z2->Z3 or Z4->Z3
                // (pivot.minBlocking, elevator.maxBlocking)
                State secondMid = new State(Settings.maxPositionWhereElevatorDoesNotBlockPivot,
                        Settings.minPositionWherePivotDoesNotCollideWithElevator, null, null,
                        end.spoilerSetPoint, null, () -> {
                            return (Math.abs(pivot.getPosition()
                                    - Settings.minPositionWherePivotDoesNotCollideWithElevator) < Settings.allowedPivotError)
                                    && (Math.abs(elevator.getPosition()
                                            - Settings.maxPositionWhereElevatorDoesNotBlockPivot) < Settings.allowedElevatorError);
                        });

                if (startZone < endZone) {
                    // Z1->Z2->Z3->Z4
                    // nothing to do:
                    // - firstMid = Z1->Z2
                    // - secondMid = Z2->Z3
                    result.add(firstMid);
                    result.add(secondMid);
                } else {
                    // Z4->Z3->Z2->Z1
                    // need to swap firstMid and secondMid
                    result.add(secondMid);
                    result.add(firstMid);
                }
            }
            return Optional.of(result);
        }

        private State currentGoalState() {
            return this.trajectory.peek();
        }

        private boolean isCurrentStateFinished() {
            return this.currentGoalState().finished.getAsBoolean();
        }

        private void proceedToNextState() {
            State next = this.trajectory.pop();
            System.out.printf("[GoToState] proceeding to next state: %s\n", next.toString());
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
                return;
            }

            if (goal.elevatorSetPoint != null
                    && (goal.elevatorSetPoint < Settings.minElevatorPosition
                            || goal.elevatorSetPoint > Settings.maxElevatorPosition)) {
                DriverStation.reportError("[GoToState] cannot put Elevator out of bounds", true);
                // make this command a no-op
                this.trajectory.clear();
                this.trajectory.push(new State());
                return;
            }
            if (goal.pivotSetPoint != null && (goal.pivotSetPoint < Settings.minPivotPosition
                    || goal.pivotSetPoint > Settings.maxPivotPosition)) {
                DriverStation.reportError("[GoToState] cannot put Pivot out of bounds", true);
                // make this command a no-op
                this.trajectory.clear();
                this.trajectory.push(new State());
                return;
            }
            if (goal.spoilerSetPoint != null && (goal.spoilerSetPoint < Settings.minSpoilerPosition
                    || goal.spoilerSetPoint > Settings.maxSpoilerPosition)) {
                DriverStation.reportError("[GoToState] cannot put Spoiler out of bounds", true);
                // make this command a no-op
                this.trajectory.clear();
                this.trajectory.push(new State());
                return;
            }

            // Get current state
            State curr = new State(this.elevator.getPosition(), this.pivot.getPosition(),
                    this.coral.getSpeed(), this.algae.getSpeed(), this.spoiler.getArmPosition(),
                    this.spoiler.getRollerSpeed(), null); // done == null b/c we're already finished
            State fullGoal = goal.merge(curr);

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

            int startZone = zone(curr);
            int endZone = zone(fullGoal);
            if (startZone == 0 || endZone == 0) {
                DriverStation.reportError(String.format(
                        "Could not determine State zones for %s->%s (got zones %d->%d)",
                        curr.toString(), fullGoal.toString(), startZone, endZone), false);
                // make this command a no-op
                this.trajectory.clear();
                this.trajectory.push(new State());
                return;
            }
            int extraStates = Math.abs(startZone - endZone) - 1;
            if (extraStates > 2) {
                DriverStation
                        .reportError(
                                String.format("Invalid zones for States %s->%s (got zones %d->%d)",
                                        curr.toString(), fullGoal.toString(), startZone, endZone),
                                false);
                // make this command a no-op
                this.trajectory.clear();
                this.trajectory.push(new State());
                return;
            }
            if (extraStates == 1) {
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
                        pivotPos = goal.pivotSetPoint;
                        elevatorPos = Settings.minPositionWhereElevatorDoesNotBlockPivot;
                    } else {
                        // Z2 -> Z4
                        // Z3 intermediate (pivot.minBlocking, goal.elevator)
                        pivotPos = Settings.minPositionWherePivotDoesNotCollideWithElevator;
                        elevatorPos = goal.elevatorSetPoint;
                    }
                } else {
                    if (startZone == 4) {
                        // Z4 -> Z2
                        // Z3 intermediate (goal.pivot, elevator.maxBlocking)
                        pivotPos = goal.pivotSetPoint;
                        elevatorPos = Settings.maxPositionWhereElevatorDoesNotBlockPivot;
                    } else {
                        // Z3 -> Z1
                        // Z2 intermediate (pivot.minBlocking, goal.elevator)
                        pivotPos = Settings.minPositionWherePivotDoesNotCollideWithElevator;
                        elevatorPos = goal.elevatorSetPoint;
                    }
                }
                BooleanSupplier done = () -> {
                    return (Math
                            .abs(this.pivot.getPosition() - pivotPos) < Settings.allowedPivotError)
                            && (Math.abs(this.elevator.getPosition()
                                    - elevatorPos) < Settings.allowedElevatorError);
                };
                State mid = new State(elevatorPos, pivotPos, null, null, goal.spoilerSetPoint, null,
                        done);
                this.trajectory.push(mid);
            } else {
                // Z1->Z2->Z3->Z4 (or reverse)

                // Z1->Z2 or Z3->Z2
                // (pivot.minBlocking, elevator.minBlocking)
                State firstMid = new State(Settings.minPositionWhereElevatorDoesNotBlockPivot,
                        Settings.minPositionWherePivotDoesNotCollideWithElevator, null, null,
                        goal.spoilerSetPoint, null, () -> {
                            return (Math.abs(this.pivot.getPosition()
                                    - Settings.minPositionWherePivotDoesNotCollideWithElevator) < Settings.allowedPivotError)
                                    && (Math.abs(this.elevator.getPosition()
                                            - Settings.minPositionWhereElevatorDoesNotBlockPivot) < Settings.allowedElevatorError);
                        });

                // Z2->Z3 or Z4->Z3
                // (pivot.minBlocking, elevator.maxBlocking)
                State secondMid = new State(Settings.maxPositionWhereElevatorDoesNotBlockPivot,
                        Settings.minPositionWherePivotDoesNotCollideWithElevator, null, null,
                        goal.spoilerSetPoint, null, () -> {
                            return (Math.abs(this.pivot.getPosition()
                                    - Settings.minPositionWherePivotDoesNotCollideWithElevator) < Settings.allowedPivotError)
                                    && (Math.abs(this.elevator.getPosition()
                                            - Settings.maxPositionWhereElevatorDoesNotBlockPivot) < Settings.allowedElevatorError);
                        });

                if (startZone < endZone) {
                    // Z1->Z2->Z3->Z4
                    // nothing to do:
                    // - firstMid = Z1->Z2
                    // - secondMid = Z2->Z3
                    this.trajectory.push(secondMid); // pushing in reverse b/c this.trajectory is a Stack
                    this.trajectory.push(firstMid);
                } else {
                    // Z4->Z3->Z2->Z1
                    // need to swap firstMid and secondMid
                    this.trajectory.push(firstMid); // pushing in reverse b/c this.trajectory is a Stack
                    this.trajectory.push(secondMid);
                }
            }
            System.out.printf("[GoToState] generated trajectory:\n%s\n", this.trajectory);
        }

        @Override
        public void execute() {
            if (this.isCurrentStateFinished()) {
                this.proceedToNextState();
            }
        }

        @Override
        public void end(boolean interrupted) {
            System.out.printf("[GoToState] ended (interrupted:%b)\n", interrupted);
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

    public static Command Feed(Elevator elevator, Pivot pivot, CoralHandler coral,
            AlgaeHandler algae, AlgaeFloorIntake spoiler) {

        // also default position w/o Coral
        BooleanSupplier done = () -> {
            return (Math.abs(elevator.getPosition() - Settings.ELEVATOR_FEED_POSITION) < 1)
                    && (Math.abs(pivot.getPosition() - Settings.PIVOT_FEED_POSITION) < 1);
        };
        State goal = new State(Settings.ELEVATOR_FEED_POSITION, Settings.PIVOT_FEED_POSITION, null,
                null, null, null, done);
        return new GoToState(goal, elevator, pivot, coral, algae, spoiler).withName("Feed");
    }

    public static Command L1(Elevator elevator, Pivot pivot, CoralHandler coral, AlgaeHandler algae,
            AlgaeFloorIntake spoiler) {

        BooleanSupplier done = () -> {
            return (Math.abs(elevator.getPosition() - Settings.ELEVATOR_L1_POSITION) < 1)
                    && (Math.abs(pivot.getPosition() - Settings.PIVOT_L1_POSITION) < 1);
        };
        State goal = new State(Settings.ELEVATOR_L1_POSITION, Settings.PIVOT_L1_POSITION, null,
                null, null, null, done);
        return new GoToState(goal, elevator, pivot, coral, algae, spoiler).withName("L1");
    }

    public static Command L2(Elevator elevator, Pivot pivot, CoralHandler coral, AlgaeHandler algae,
            AlgaeFloorIntake spoiler) {

        BooleanSupplier done = () -> {
            return (Math.abs(elevator.getPosition() - Settings.ELEVATOR_L2_POSITION) < 1)
                    && (Math.abs(pivot.getPosition() - Settings.PIVOT_L2_POSITION) < 1);
        };
        State goal = new State(Settings.ELEVATOR_L2_POSITION, Settings.PIVOT_L2_POSITION, null,
                null, null, null, done);
        return new GoToState(goal, elevator, pivot, coral, algae, spoiler).withName("L2");
    }

    public static Command L3(Elevator elevator, Pivot pivot, CoralHandler coral, AlgaeHandler algae,
            AlgaeFloorIntake spoiler) {

        BooleanSupplier done = () -> {
            return (Math.abs(elevator.getPosition() - Settings.ELEVATOR_L3_POSITION) < 1)
                    && (Math.abs(pivot.getPosition() - Settings.PIVOT_L3_POSITION) < 1);
        };
        State goal = new State(Settings.ELEVATOR_L3_POSITION, Settings.PIVOT_L3_POSITION, null,
                null, null, null, done);
        return new GoToState(goal, elevator, pivot, coral, algae, spoiler).withName("L3");
    }

    public static Command L4(Elevator elevator, Pivot pivot, CoralHandler coral, AlgaeHandler algae,
            AlgaeFloorIntake spoiler) {

        BooleanSupplier done = () -> {
            return (Math.abs(elevator.getPosition() - Settings.ELEVATOR_L4_POSITION) < 1)
                    && (Math.abs(pivot.getPosition() - Settings.PIVOT_L4_POSITION) < 1);
        };
        State goal = new State(Settings.ELEVATOR_L4_POSITION, Settings.PIVOT_L4_POSITION, null,
                null, null, null, done);
        return new GoToState(goal, elevator, pivot, coral, algae, spoiler).withName("L4");
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
