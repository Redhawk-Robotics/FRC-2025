package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import java.util.function.BiConsumer;
import java.util.function.Supplier;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds =
            new SwerveRequest.ApplyRobotSpeeds();

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
            new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
            new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
            new SwerveRequest.SysIdSwerveRotation();

    /*
     * SysId routine for characterizing translation. This is used to find PID gains for the drive
     * motors.
     */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_translationCharacterization.withVolts(output)), null,
                    this));

    /*
     * SysId routine for characterizing steer. This is used to find PID gains for the steer motors.
     */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(new SysIdRoutine.Config(null, // Use
            // default
            // ramp
            // rate
            // (1
            // V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this));

    /*
     * SysId routine for characterizing rotation. This is used to find PID gains for the
     * FieldCentricFacingAngle HeadingController. See the documentation of
     * SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(new SysIdRoutine.Config(
            /*
             * This is in radians per second^2, but SysId only supports "volts per second"
             */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI), null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            }, null, this));

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    public enum speeds {
        NINETY_PERCENT(1), //
        EIGHTY_PERCENT(1), //
        SIXTY_PERCENT(0.6), //
        FOURTY_PERCENT(0.4), //
        TWENTY_PERCENT(0.2), //
        MIN(0.1);

        private final double m;

        private speeds(double multiplier) {
            this.m = multiplier;
        }

        public double mult() {
            return this.m;
        }

        public String toString() {
            switch (this) {
                case NINETY_PERCENT:
                    return "90% SPEED";
                case EIGHTY_PERCENT:
                    return "80% SPEED";
                case SIXTY_PERCENT:
                    return "60% SPEED";
                case FOURTY_PERCENT:
                    return "40% SPEED";
                case TWENTY_PERCENT:
                    return "20% SPEED";
                case MIN:
                    return ".05% SPEED";
                default:
                    return "<unk?>";
            }
        }
    }

    private speeds m_speedMultiplier = speeds.EIGHTY_PERCENT;


    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct the devices
     * themselves. If they need the devices, they can access them through getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules Constants for each specific module
     */
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        // config AutoBuilder
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct the devices
     * themselves. If they need the devices, they can access them through getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set
     *        to 0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
     * @param modules Constants for each specific module
     */
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct the devices
     * themselves. If they need the devices, they can access them through getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set
     *        to 0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation in the form
     *        [x, y, theta]^T, with units in meters and radians
     * @param visionStandardDeviation The standard deviation for vision calculation in the form [x,
     *        y, theta]^T, with units in meters and radians
     * @param modules Constants for each specific module
     */
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency, Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation,
                visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    private void configureAutoBuilder() {
        try {
            RobotConfig config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(//
                    this::getPose, // Supplier of current robot pose
                    this::resetPose, // Consumer for seeding pose against auto
                    this::getSpeeds, // Supplier of current robot speeds
                    // Consumer of ChassisSpeeds and feedforwards to drive the robot
                    (speeds, feedforwards) -> setControl(m_pathApplyRobotSpeeds.withSpeeds(speeds)
                            .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                            .withWheelForceFeedforwardsY(
                                    feedforwards.robotRelativeForcesYNewtons())),
                    new PPHolonomicDriveController( // TODO tune this! These are _okay_ for now
                            new PIDConstants(1.5, .5, 0.25), // PID constants for translation
                            new PIDConstants(1.25, 0, 0)), // PID constants for rotation
                    config,
                    // Flip the path if Alliance is Red
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red, //
                    this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder",
                    ex.getStackTrace());
        }
    }

    public Pose2d getPose() {
        return this.getState().Pose;
    }

    public ChassisSpeeds getSpeeds() {
        return this.getState().Speeds;
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine specified by
     * {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine specified by
     * {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    public void increaseSpeedMultiplier() {
        switch (this.m_speedMultiplier) {
            case MIN:
                this.m_speedMultiplier = speeds.TWENTY_PERCENT;
            case TWENTY_PERCENT:
                this.m_speedMultiplier = speeds.FOURTY_PERCENT;
            case FOURTY_PERCENT:
                this.m_speedMultiplier = speeds.SIXTY_PERCENT;
            case SIXTY_PERCENT:
                this.m_speedMultiplier = speeds.EIGHTY_PERCENT;
            case EIGHTY_PERCENT:
                this.m_speedMultiplier = speeds.NINETY_PERCENT;
            default:
                break;
            // case SLOW:
            //     this.m_speedMultiplier = speeds.TWENTY_PERCENT;
            //     break;
            // case TWENTY_PERCENT:
            //     this.m_speedMultiplier = speeds.FAST;
            //     break;
            // default:
            //     break;
        }
        System.out.printf("Drive speed multiplier is now %f\n", this.speedMultiplier());
    }

    public void decreaseSpeedMultiplier() {
        switch (this.m_speedMultiplier) {
            // case MIN:
            //     this.m_speedMultiplier = speeds.TWENTY_PERCENT;
            case TWENTY_PERCENT:
                this.m_speedMultiplier = speeds.MIN;
            case FOURTY_PERCENT:
                this.m_speedMultiplier = speeds.TWENTY_PERCENT;
            case SIXTY_PERCENT:
                this.m_speedMultiplier = speeds.FOURTY_PERCENT;
            case EIGHTY_PERCENT:
                this.m_speedMultiplier = speeds.SIXTY_PERCENT;
            case NINETY_PERCENT:
                this.m_speedMultiplier = speeds.EIGHTY_PERCENT;
            default:
                break;
            // case FAST:
            //     this.m_speedMultiplier = speeds.TWENTY_PERCENT;
            //     break;
            // case TWENTY_PERCENT:
            //     this.m_speedMultiplier = speeds.SLOW;
            //     break;
            // default:
            //     break;
        }
        System.out.printf("Drive speed multiplier is now %f\n", this.speedMultiplier());
    }

    public double speedMultiplier() {
        return this.m_speedMultiplier.mult();
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective. If we haven't applied the operator
         * perspective before, then we should apply it regardless of DS state. This allows us to
         * correct the perspective in case the robot code restarts mid-match. Otherwise, only check
         * and apply the operator perspective if the DS is disabled. This ensures driving behavior
         * doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }

        SmartDashboard.putString("Drive/speedMultiplier", this.m_speedMultiplier.toString());
        SmartDashboard.putNumber("Drive/speedMultiplierVal", this.m_speedMultiplier.mult());
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);

    }
}
