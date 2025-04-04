package frc.robot.Constants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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
            public static final int ID_BOTTOM_RIGHT = 2;
            public static final int ID_TOP_LEFT = 3;
            public static final int ID_BOTTOM_LEFT = 4;
        }

        public static final MotorType TOP_RIGHT_MOTORTYPE = MotorType.kBrushless;
        public static final MotorType BOTTOM_RIGHT_MOTORTYPE = MotorType.kBrushless;
        public static final MotorType TOP_LEFT_MOTORTYPE = MotorType.kBrushless;
        public static final MotorType BOTTOM_LEFT_MOTORTYPE = MotorType.kBrushless;

        public static final boolean TOP_LEFT_INVERT = true;
        public static final boolean BOTTOM_LEFT_INVERT = true;

        public static final int CURRENT_LIMIT = 60;
        public static final IdleMode IDLE_MODE = IdleMode.kBrake;

        // elevator max == 32
        public static final double CONVERSION_FACTOR = 100. / 32.;
        public static final double kP_UP = 0.050;
        public static final double kI_UP = 0.;
        public static final double kD_UP = 0.;
        public static final double kP_DOWN = 0.0199;
        public static final double kI_DOWN = 0.;
        public static final double kD_DOWN = 0.;
    }

    public static final class Pivot {
        public static final class CAN {
            public static final int ID_LEFT_MOTOR = 5;
        }

        public static final MotorType LEFT_PIVOT_MOTORTYPE = MotorType.kBrushless;

        public static final double ZERO_OFFSET = 0.1969758;
        public static final double CONVERSION_FACTOR = 1.;
        public static final double kP = 0.025;
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
        public static final double kCAMERAPOSE_YAW = 72.5; // DOUBLE CHECK  MIGHT BE EIGHT DEGREES, NEW MOUNT IS CLOSER TO 0
        public static final double kCAMERAPOSE_FORWARD = .1905;//0.24;
        public static final double kCAMERAPOSE_SIDE = -0.2286; //-0.28;
        public static final double kCAMERAPOSE_UP = 0.4953; //0.32;
    }

    public static final class CoralAligner {
        public static final class CAN {
            public static final int kLEFT = 0;
            public static final int kRIGHT = 0;
            public static final String kBUS = "rio";
        }

        public static final class RANGE {
            public static final double kLOWERBOUND = 0; //TODO TO BE TUNED
        }
    }
}
