package frc.robot.Constants;

public interface Ports {

    public static class Gamepad {
        public static int DRIVER = 0;
<<<<<<< Updated upstream
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
        public static int PIVOT = 0;
        public static int WHEEL_INTAKE = 0;

        public static boolean PIVOT_INVERTED = false;
        public static boolean WHEEL_INTAKE_INVERTED = false;

=======
        public static int OPERATOR = 1;
    }

    public static final class Elevator {
        public static int TOP_RIGHT = 1;
        public static int BOTTOM_RIGHT = 2;
        public static int TOP_LEFT = 3;
        public static int BOTTOM_LEFT = 4;

    }

    public static final class CoralIntake {
        public static int PIVOT_RIGHT = 0;
        public static int PIVOT_LEFT = 0;
        public static int WHEEL_INTAKE = 0;

>>>>>>> Stashed changes
    }

    public static final class LIGHTS {
        public static int LED1 = 0;
    }
}