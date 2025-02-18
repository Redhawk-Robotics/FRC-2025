package frc.robot.Constants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public final class Settings {

    public static final class Elevator {
        public static MotorType TOP_RIGHT_MOTORTYPE = MotorType.kBrushless;
        public static MotorType BOTTOM_RIGHT_MOTORTYPE = MotorType.kBrushless;

        public static MotorType TOP_LEFT_MOTORTYPE = MotorType.kBrushless;
        public static MotorType BOTTOM_LEFT_MOTORTYPE = MotorType.kBrushless;

        public static boolean TOP_LEFT_INVERT = true;
        public static boolean BOTTOM_LEFT_INVERT = true;

        public static int CURRENT_LIMIT = 60;
        public static IdleMode IDLE_MODE = IdleMode.kBrake;

    }

    public static final class Pivot {
        public static MotorType LEFT_PIVOT_MOTORTYPE = MotorType.kBrushless;
        public static MotorType RIGHT_PIVOT_MOTORTYPE = MotorType.kBrushless;
    }

    public static final class AlgaeHandler {

        public static MotorType INTAKE_MOTORTYPE = MotorType.kBrushless;
        public static MotorType ROLLER_MOTORTYPE = MotorType.kBrushless;
    }

}
