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

        
    }

    public static final class CoralIntake {
        public static int PIVOT_LEFT = 0;
        public static int PIVOT_RIGHT = 0;
        public static int WHEEL_INTAKE = 0;

    }

    public static final class LIGHTS {
        public static int LED1 = 0;
    }

    public static final class AlgaeHandler {
        public static int kCANID_INTAKE = 0;
        public static int kCANID_ROLLER = 0;
    }
}
