// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.Consumer;
import java.util.function.Supplier;
import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Settings;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;

public final class CoralPositionFactory {

    // elevator goes from 43.5in to 74in ~ 30in of range
    // elevator setPosition reference is configured to range from 0 to 30
    // using the absoluteEncoder config offset/conversions
    // actually, maybe it'd be better to do [0,100]

    // pivot goes from normal 270deg to 90deg ~ 180deg of range
    // pivot config is similar to the elevator
    // the config changes this to be 0 to 180deg
    // actually, maybe it'd be better to do [0,100]

    private static enum position {
        F0(Settings.CoralPosition.ELEVATOR_FEED_POSITION,
                Settings.CoralPosition.PIVOT_FEED_POSITION), //
        L1(Settings.CoralPosition.ELEVATOR_L1_POSITION, Settings.CoralPosition.PIVOT_L1_POSITION), //
        L2(Settings.CoralPosition.ELEVATOR_L2_POSITION, Settings.CoralPosition.PIVOT_L2_POSITION), //
        L3(Settings.CoralPosition.ELEVATOR_L3_POSITION, Settings.CoralPosition.PIVOT_L3_POSITION), //
        L4(Settings.CoralPosition.ELEVATOR_L4_POSITION, Settings.CoralPosition.PIVOT_L4_POSITION);

        private final double elevPos;
        private final double pivotPos;

        private position(double e, double p) {
            this.elevPos = e;
            this.pivotPos = p;
        }

        public double elevatorPosition() {
            return this.elevPos;
        }

        public double pivotPosition() {
            return this.pivotPos;
        }

        public String toString() {
            switch (this) {
                case F0:
                    return "Feed<>";
                case L1:
                    return "L1<>";
                case L2:
                    return "L2<>";
                case L3:
                    return "L3<>";
                case L4:
                    return "L4<>";
            }
            return "<??>";
        }
    }


    // E_c == current Elevator position
    // E_w == wanted Elevator position
    // P_c == current Pivot position
    // P_w == wanted Pivot position
    // E_0 == bottom of the range where the Pivot blocks the Elevator movement
    // P_0 == same, but Pivot position
    // E_1 == top of the range where the Pivot blocks the Elevator movement
    // P_1 == same, but Pivot position

    // 

    // Two ranges overlap iff they do _not_ overlap
    // Two ranges do not overlap if one of them starts after the other one ends
    // dont_overlap = (x2 < y1 || x1 > y2)
    // overlap = !(x2 < y1 || x1 > y2)
    // = (x2 >= y1 && x1 <= y2)
    // [x1,x2] [y1,y2]
    // https://stackoverflow.com/a/35754308
    // For us, [y1,y2] = [E_0,E_1]
    // and [x1,x2] = [E_c,E_w] or [E_w,E_c] depending
    // on the intended Elevator direction
    //
    // If the ranges [E_0,E_1] [E_c,E_w] overlap
    // then the Pivot has to go to the safe zone
    // This only applies when the Pivot is in [P_0,P_1]
    // ie when [P_0,P_1] [P_c,P_w] overlap

    private static boolean overlap(double x1, double x2, double y1, double y2) {
        return x2 >= y1 && x1 <= y2;
    }

    private static boolean elevatorWilBlock(double Ec, double Ew) {
        if (Ec > Ew) {
            return overlap(Ew, Ec, Settings.CoralPosition.ELEVATOR_MIN_BLOCKING_POSITION,
                    Settings.CoralPosition.ELEVATOR_MAX_BLOCKING_POSITION);
        }
        return overlap(Ec, Ew, Settings.CoralPosition.ELEVATOR_MIN_BLOCKING_POSITION,
                Settings.CoralPosition.ELEVATOR_MAX_BLOCKING_POSITION);
    }

    private static boolean pivotWillBlock(double Pc, double Pw) {
        if (Pc > Pw) {
            return overlap(Pw, Pc, Settings.CoralPosition.PIVOT_MIN_BLOCKING_POSITION,
                    Settings.CoralPosition.PIVOT_MAX_BLOCKING_POSITION);
        }
        return overlap(Pc, Pw, Settings.CoralPosition.PIVOT_MIN_BLOCKING_POSITION,
                Settings.CoralPosition.PIVOT_MAX_BLOCKING_POSITION);
    }

    private static Command setAndWait(String system, position location, double position,
            double error, Command setPosition, Supplier<Double> getPosition) {

        return setPosition.andThen(//
                Commands.race(//
                        Commands.runOnce(() -> {
                            System.out.printf("Waiting for '%s' to get to position %f for %s...\n",
                                    system, position, location.toString());
                        }).andThen(//
                                Commands.waitSeconds(1)//
                        ).repeatedly().until(() -> {
                            return Math.abs(getPosition.get() - position) < error;
                        }).andThen(() -> {
                            System.out.printf("At position %s!\n", location.toString());
                        }),
                        //
                        Commands.waitSeconds(5).andThen(Commands.runOnce(() -> {
                            System.out.printf("setAndWait timed out for %s for %s\n", system,
                                    location.toString());
                        })))//
        ).andThen(Commands.runOnce(() -> {
            System.out.printf("%s to %s done.\n", system, location.toString());
        }));
    }

    private static Command orchestrate(Elevator elevator, Pivot pivot, position p) {
        Command makePivotSafe = Commands.either(//
                Commands.sequence(
                        setAndWait("Pivot", p, Settings.CoralPosition.PIVOT_MAX_BLOCKING_POSITION,
                                Settings.CoralPosition.PIVOT_ALLOWED_ERROR,
                                pivot.runOnce(() -> pivot.setReference(
                                        Settings.CoralPosition.PIVOT_MAX_BLOCKING_POSITION)),
                                pivot::getPosition),
                        setAndWait("Elevator", p,
                                Settings.CoralPosition.ELEVATOR_MIN_BLOCKING_POSITION,
                                Settings.CoralPosition.ELEVATOR_ALLOWED_ERROR,
                                elevator.runOnce(() -> elevator.setReference(
                                        Settings.CoralPosition.ELEVATOR_MIN_BLOCKING_POSITION)),
                                elevator::getPosition)),
                Commands.runOnce(() -> {
                    System.out.println("no overlap detected"); // TODO fix this
                }), //
                () -> {
                    double Ec = elevator.getPosition(), Ew = p.elevatorPosition();
                    double Pc = elevator.getPosition(), Pw = p.pivotPosition();
                    System.out.printf("Elevator %f->%f;\nPivot %f->%f\n", Ec, Ew, Pc, Pw);
                    return pivotWillBlock(Pc, Pw) || elevatorWilBlock(Ec, Ew);
                });

        Command result = Commands.sequence(//
                makePivotSafe,
                Commands.parallel(
                        setAndWait("Pivot", p, p.pivotPosition(),
                                Settings.CoralPosition.PIVOT_ALLOWED_ERROR,
                                pivot.runOnce(() -> pivot.setReference(p.pivotPosition())),
                                pivot::getPosition),
                        setAndWait("Elevator", p, p.elevatorPosition(),
                                Settings.CoralPosition.ELEVATOR_ALLOWED_ERROR,
                                elevator.runOnce(() -> elevator.setReference(p.elevatorPosition())),
                                elevator::getPosition)));
        result.addRequirements(pivot, elevator);
        return result;
    }

    public static Command Feed(Elevator elevator, Pivot pivot) {
        // also default position w/o Coral
        return orchestrate(elevator, pivot, position.F0);
    }

    public static Command L1(Elevator elevator, Pivot pivot) {
        // you could use this to test out just moving both systems directly to L1
        // Command result =
        //         Commands.parallel(elevator.setReferenceRequest(position.L1::elevatorPosition),
        //                 pivot.setReferenceRequest(position.L1::pivotPosition));
        // result.addRequirements(pivot, elevator);
        // return result;
        return orchestrate(elevator, pivot, position.L1);
    }

    public static Command L2(Elevator elevator, Pivot pivot) {
        return orchestrate(elevator, pivot, position.L2);
    }

    public static Command L3(Elevator elevator, Pivot pivot) {
        return orchestrate(elevator, pivot, position.L3);
    }

    public static Command L4(Elevator elevator, Pivot pivot) {
        return orchestrate(elevator, pivot, position.L4);
    }

    // this might be useful?
    // public static Command Stop(Elevator elevator, Pivot pivot) {
    //     Command result = Commands.parallel(//
    //             elevator.runOnce(() -> elevator.stopElevator()),
    //             pivot.runOnce(() -> pivot.stopPivot()));
    //     result.addRequirements(pivot, elevator);
    //     return result;
    // }
}
