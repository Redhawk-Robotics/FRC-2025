package frc.robot.Constants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public final class Settings {

    public static final class Elevator {
        public static final MotorType TOP_RIGHT_MOTORTYPE = MotorType.kBrushless;
        public static final MotorType BOTTOM_RIGHT_MOTORTYPE = MotorType.kBrushless;

        public static final MotorType TOP_LEFT_MOTORTYPE = MotorType.kBrushless;
        public static final MotorType BOTTOM_LEFT_MOTORTYPE = MotorType.kBrushless;

        public static final int CURRENT_LIMIT = 60;
        public static final IdleMode IDLE_MODE = IdleMode.kBrake;

        // elevator max == 32
        public static final double CONVERSION_FACTOR = 100. / 32.; // TODO remove this, and re-tune PID
        public static final double kP_UP = 0.050;
        public static final double kI_UP = 0.;
        public static final double kD_UP = 0.;
        public static final double kP_DOWN = 0.0199; // TODO tune this
        public static final double kI_DOWN = 0.;
        public static final double kD_DOWN = 0.;
    }

    public static final class Pivot {
        public static final MotorType LEFT_PIVOT_MOTORTYPE = MotorType.kBrushless;

        public static final double ZERO_OFFSET = 0.8; // TODO remove this, and re-tune PID
        public static final double CONVERSION_FACTOR = 100. / 0.5; // TODO remove this, and re-tune PID
        public static final double kP = 0.025;
        public static final double kI = 0.;
        public static final double kD = 0.;
    }

    public static final class CoralPosition {
        // TODO ALL OF THESE NEED TO BE TUNED ALONGSIDE PID
        // these values change depending on the zeroOffset and conversionFactor
        public static final double ELEVATOR_MIN_BLOCKING_POSITION = 5.; // elevator blocks the pivot above this reference
        public static final double ELEVATOR_MAX_BLOCKING_POSITION = 55.; // elevator blocks the pivot below this reference
        public static final double PIVOT_MIN_BLOCKING_POSITION = 65; // for completeness // ! SWAPPED TO TEST
        public static final double PIVOT_MAX_BLOCKING_POSITION = 120.; // elevator can move in the range above when pivot is here
        public static final double ELEVATOR_ALLOWED_ERROR = 1;
        public static final double PIVOT_ALLOWED_ERROR = 1;
        public static final double ELEVATOR_FEED_POSITION = 0;
        public static final double ELEVATOR_L1_POSITION = ELEVATOR_FEED_POSITION;
        public static final double ELEVATOR_L2_POSITION = ELEVATOR_FEED_POSITION;
        public static final double ELEVATOR_L3_POSITION = ELEVATOR_FEED_POSITION;
        public static final double ELEVATOR_L4_POSITION = 106;
        public static final double PIVOT_FEED_POSITION = 65;
        public static final double PIVOT_L1_POSITION = 145;
        public static final double PIVOT_L2_POSITION = 153;
        public static final double PIVOT_L3_POSITION = 175; // pivot L2=L3
        public static final double PIVOT_L4_POSITION = 176;
    }

    public static final class CoralHandler {
        public static final MotorType CORAL_INTAKE_MOTORTYPE = MotorType.kBrushless;
        //TODO IDENTIFY THE RANGE
        public static final double CORAL_INTAKE_VOLTAGE = 10.8; // If the voltage is less than this, triggered by  intaking
        public static final double CORAL_OUTTAKE_VOLTAGE = 11.2; // IF the voltage is more than this, triggered by outtake

    }

    public static final class AlgaeHandler {
        public static final MotorType ALGAE_INTAKE_MOTORTYPE = MotorType.kBrushless;
    }

    public static final class AlgaeFloorIntake{
        public static final MotorType ALGAE_FLOOR_INTAKE_MOTORTYPE = MotorType.kBrushless;
        public static final double HOME_ENCODER_POSITION = 0;
        public static final double SAFE_TO_TRAVEL_POSITION = 0;
        public static final double FLOOR_INTAKING_POSITION = 0;
    }

    public static final class LimeLight {
        public static final double kCAMERAPOSE_YAW = 72.5;
        public static final double kCAMERAPOSE_FORWARD = 0.24;
        public static final double kCAMERAPOSE_SIDE = -0.28;
        public static final double kCAMERAPOSE_UP = 0.32;
    }

    //& APRIL TAGS TBD 




}
