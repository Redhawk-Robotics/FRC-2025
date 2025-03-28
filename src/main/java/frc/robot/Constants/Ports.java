package frc.robot.Constants;

public final class Ports {

    public static class Gamepad {
        public static int DRIVER = 0;
        public static int OPERATOR = 1;
    }

    public static final class Elevator {
        public static int kCAN_ID_TOP_RIGHT = 1;
        public static int kCAN_ID_BOTTOM_RIGHT = 2;
        public static int kCAN_ID_TOP_LEFT = 3;
        public static int kCAN_ID_BOTTOM_LEFT = 4;

        public static boolean TOP_LEFT_INVERT = true;
        public static boolean BOTTOM_LEFT_INVERT = true;
    }

    public static final class CoralIntake {
        public static int PIVOT_LEFT = 5;
        public static int PIVOT_RIGHT = 5;
        public static int WHEEL_INTAKE = 7;

        public static boolean PIVOT_INVERTED = false;
        public static boolean WHEEL_INTAKE_INVERTED = false;

    }

    public static final class LIGHTS {
        public static int LED1 = 0;
    }

    public static final class AlgaeHandler {
        public static int ALGAEINTAKE_MOTOR = 6;
    }

    public static final class Climber {
        public static int kCAN_ID_CLIMBER = 10;
    }
    public static final class AlgaeFloorIntake{
        public static int kCAN_ID_ROLLER = 12;
        public static int kCAN_ID_LEFT = 11;

    }
}

