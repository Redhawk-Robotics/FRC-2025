package frc.robot.Constants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public final class Settings {

    public static final class Elevator {

        public static double GEAR_RATIO = 0;

        public static MotorType TOP_RIGHT_MOTORTYPE = MotorType.kBrushless;
        public static MotorType BOTTOM_RIGHT_MOTORTYPE = MotorType.kBrushless;

        public static MotorType TOP_LEFT_MOTORTYPE = MotorType.kBrushless;
        public static MotorType BOTTOM_LEFT_MOTORTYPE = MotorType.kBrushless;

        public static boolean TOP_LEFT_INVERT = true;
        public static boolean BOTTOM_LEFT_INVERT = true;

        public static int CURRENT_LIMIT = 60;
        public static IdleMode IDLE_MODE = IdleMode.kBrake;

        // TODO FIGURE OUT THIS
        public static int MAXMOTION_SETPOINT = 0;

        // TODO FIGURE OUT THESE
        public static int CLOSEDLOOP_kP  = 0;
        public static int CLOSEDLOOP_kI  = 0;
        public static int CLOSEDLOOP_kD  = 0;

        // TODO FIGURE OUT THESE 
        public static int VELOCITY_kP  = 0;
        public static int VELOCITY_kI  = 0;
        public static int VELOCITY_kD  = 0;

        public static int HOME_SETPOINT = 0;
        public static int FEEDERSTATION_SETPOINT = 0;
        public static int L1_SETPOINT = 0;
        public static int L2_SETPOINT = 0;
        public static int L3_SETPOINT = 0;
        public static int L4_SETPOINT = 0;

    }

    public static final class Pivot {
        public static MotorType LEFT_PIVOT_MOTORTYPE = MotorType.kBrushless;
        public static MotorType RIGHT_PIVOT_MOTORTYPE = MotorType.kBrushless;
    }

}
