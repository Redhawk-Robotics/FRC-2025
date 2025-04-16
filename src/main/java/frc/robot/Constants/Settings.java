package frc.robot.Constants;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public final class Settings {
    public static final class Gamepad {
        public static final int DRIVER = 0;
        public static final int OPERATOR = 1;
    }

    public static final class Elevator {
        // looking at the elevator with the motors in view
        // looking at the front of the robot
        public static final class CAN {
            public static final int ID_TOP_RIGHT = 1;
            // public static final int ID_BOTTOM_RIGHT = 2;
            public static final int ID_TOP_LEFT = 3;
            public static final int ID_BOTTOM_LEFT = 4;
        }

        // TODO tune these (and kG and kS)
        public static final double kG = 28;
        public static final double kS = 15;
        public static final double kP = 15;
        public static final double kI = 0;
        public static final double kD = 0;
    }

    public static final class Pivot {
        public static final class CAN {
            public static final int ID_LEFT_MOTOR = 5;
        }

        public static final MotorType LEFT_PIVOT_MOTORTYPE = MotorType.kBrushless;

        public static final double ZERO_OFFSET = 0.1969758;
        public static final double CONVERSION_FACTOR = 1.;
        public static final double kP = 10.;
        public static final double kI = 0.;
        public static final double kD = 0.;
    }

    public static final class AlgaeHandler {
        public static final class CAN {
            public static final int ID_MOTOR = 6;
        }

        public static final MotorType ALGAE_INTAKE_MOTORTYPE = MotorType.kBrushless;
    }

    public static final class CoralHandler {
        public static final class CAN {
            public static final int ID_WHEEL_INTAKE = 7;
        }

        public static final MotorType CORAL_INTAKE_MOTORTYPE = MotorType.kBrushless;
        //TODO IDENTIFY THE RANGE
        public static final double CORAL_INTAKE_VOLTAGE = 10.8; // If the voltage is less than this, triggered by  intaking
        public static final double CORAL_OUTTAKE_VOLTAGE = 11.2; // IF the voltage is more than this, triggered by outtake

        public static final boolean PIVOT_INVERTED = false; // TODO used?
        public static final boolean WHEEL_INTAKE_INVERTED = false; // TODO used?
    }

    public static final class Climber {
        public static final class CAN {
            public static final int ID_CLIMBER = 10;
        }
    }

    public static final class AlgaeFloorIntake {
        public static final class CAN {
            public static final int ID_ARM = 11;
            public static final int ID_ROLLER = 12;
        }

        public static final MotorType ALGAE_FLOOR_INTAKE_MOTORTYPE = MotorType.kBrushless;
        public static final double HOME_ENCODER_POSITION = 0;
        public static final double SAFE_TO_TRAVEL_POSITION = 0;
        public static final double FLOOR_INTAKING_POSITION = 0;
    }

    public static final class LimeLight {
        // public static final double kCAMERAPOSE_YAW = 167; // DOUBLE CHECK  MIGHT BE EIGHT DEGREES, NEW MOUNT IS CLOSER TO 0
        // public static final double kCAMERAPOSE_FORWARD =  -0.1178;//0.24;
        // public static final double kCAMERAPOSE_SIDE = 0.2667; //-0.28;
        // public static final double kCAMERAPOSE_UP = 0.4953; //0.32;
    }

    public static final class CoralAligner {
        public static final class CAN {
            public static final int kLEFT = 20;
            public static final int kRIGHT = 21;
            public static final String kBUS = "drive-bus";
        }

        public static final class RANGE {
            public static final double kLOWERBOUND = 0; //TODO TO BE TUNED
        }
    }

    public static final class Positioner {
        // conflicts:
        // - if elevator is below [threshold_e_s] (near the bottom), then the spoiler and pivot can collide
        //     when spoiler is below [threshold_s_p] and pivot is below [threshold_p_s]
        // - pivot cannot be below [threshold_p_e] if elevator is, or moves, between [threshold_e_p1, threshold_e_p2]
        // - elevator must be in range [min_e, max_e]
        // - pivot must be in range [min_p, max_p]
        // - spoiler must be in range [min_s, max_s]

        // public static final double minPositionWhereElevatorFreesPivotAndSpoiler = 4.5; // todo
        // public static final double minPositionWherePivotDoesNotCollideWithSpoiler = 10;
        // public static final double minPositionWhereSpoilerDoesNotCollideWithPivot = 5;
        public static final double minPositionWherePivotDoesNotCollideWithElevator = 0.38; // threshold_p_e (pivot blocks the elevator below this reference)
        public static final double minPositionWhereElevatorDoesNotBlockPivot = 4.5; // threshold_e_p1 (elevator blocks the pivot above this reference)
        public static final double maxPositionWhereElevatorDoesNotBlockPivot = 80; // threshold_e_p2 (elevator blocks the pivot below this reference)
        public static final double midpointPositionWhereElevatorBlocksPivot =
                (minPositionWhereElevatorDoesNotBlockPivot
                        + maxPositionWhereElevatorDoesNotBlockPivot) / 2; // (4.5+80)/2 = 42.25

        public static final double minElevatorPosition = 0;
        public static final double maxElevatorPosition = 11.5;
        public static final double allowedElevatorError = 0.05;

        public static final double minPivotPosition = 0.01;
        public static final double maxPivotPosition = 0.62;
        public static final double allowedPivotError = 0.005;

        public static final double minSpoilerPosition = 0;
        // ground position 16
        public static final double maxSpoilerPosition = 28;
        public static final double allowedSpoilerError = 0.1;

        public static final double ELEVATOR_FEED_POSITION = 0;
        public static final double ELEVATOR_ATTACK_POSITION = ELEVATOR_FEED_POSITION;
        public static final double ELEVATOR_L1_POSITION = ELEVATOR_FEED_POSITION;
        public static final double ELEVATOR_L2_POSITION = ELEVATOR_FEED_POSITION;
        public static final double ELEVATOR_L3_POSITION = 3.74;
        public static final double ELEVATOR_L4_POSITION = 10.97;
        public static final double ELEVATOR_BARGE_POSITION = ELEVATOR_L4_POSITION;
        public static final double ELEVATOR_ALGAE_L2_POSITION = ELEVATOR_FEED_POSITION;
        public static final double ELEVATOR_ALGAE_L3_POSITION = 5.72;
        public static final double ELEVATOR_ALGAE_GROUND_POSITION = 0;
        public static final double ELEVATOR_ALGAE_TRANSFER_POSITION = 0;

        public static final double PIVOT_FEED_POSITION = 0.055;
        public static final double PIVOT_ATTACK_POSITION = 0.600;
        public static final double PIVOT_L1_POSITION = PIVOT_FEED_POSITION;
        public static final double PIVOT_L2_POSITION = 0.58;
        public static final double PIVOT_L3_POSITION = 0.609;
        public static final double PIVOT_L4_POSITION = 0.58;
        public static final double PIVOT_BARGE_POSITION = 0.505;
        public static final double PIVOT_ALGAE_L2_POSITION = 0.320;
        public static final double PIVOT_ALGAE_L3_POSITION = 0.28;
        public static final double PIVOT_ALGAE_GROUND_POSITION = 0.02;
        public static final double PIVOT_ALGAE_TRANSFER_POSITION = 0.02;

        public static final double SPOILER_ALGAE_FEED_POSITION = 1;
        public static final double SPOILER_ALGAE_GROUND_POSITION = 16;
        public static final double SPOILER_ALGAE_TRANSFER_POSITION = 6;

        public static final boolean Verbose = false;
    }
}
