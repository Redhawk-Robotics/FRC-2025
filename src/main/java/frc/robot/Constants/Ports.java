package frc.robot.Constants;

public interface Ports {

    public static class Gamepad {
        public static int DRIVER = 0;
        public static int OPERATOR = 0;
    }

    public static final class Elevator {
        public static int TOP_RIGHT = 0;
        public static int BOTTOM_RIGHT = 0;
        public static int TOP_LEFT = 0;
        public static int BOTTOM_LEFT = 0;

        public static boolean TOP_LEFT_INVERT = true;
        public static boolean BOTTOM_LEFT_INVERT = true;
    }

    public static final class CoralIntake {
        public static int PIVOT_LEFT = 0;
        public static int PIVOT_RIGHT = 0;
        public static int WHEEL_INTAKE = 0;

        public static boolean PIVOT_INVERTED = false;
        public static boolean WHEEL_INTAKE_INVERTED = false;

    }

    public static final class LIGHTS {
        public static int LED1 = 0;
    }
}