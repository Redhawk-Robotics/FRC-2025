// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.AutoAlign;
import frc.robot.Commands.PlayMusic;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Commands.PositionerFactory;
import frc.robot.Constants.Settings;
import frc.robot.subsystems.Pivot;
import frc.robot.generated.TunerConstants;
import frc.robot.sendables.ControlBoard;
import frc.robot.sendables.SendablePID;
import frc.robot.subsystems.AlgaeFloorIntake;
import frc.robot.subsystems.AlgaeHandler;
import frc.robot.subsystems.CANRanges;
// import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralHandler;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.AlgaeFloorIntakeComponents.AlgaeFloorIntakeArm;
import frc.robot.subsystems.AlgaeFloorIntakeComponents.AlgaeFloorIntakeRoller;
import edu.wpi.first.math.MathUtil;

public class RobotContainer {
    // kSpeedAt12Volts desired top speed
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    // 3/4 of a rotation per second max angular velocity
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    /* Setting up bindings for necessary control of the swerve drive platform */
    // Add 5% deadband for translation/rotation controls
    // Use open-loop control for drive motors
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()//
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController DRIVER = new CommandXboxController(Settings.Gamepad.DRIVER);
    private final CommandXboxController OPERATOR =
            new CommandXboxController(Settings.Gamepad.OPERATOR);

    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final SendableChooser<Command> autoChooser;
    private final Field2d field = new Field2d();
    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
            drivetrain.getKinematics(), drivetrain.getPigeon2().getRotation2d(),
            drivetrain.getState().ModulePositions, new Pose2d()//*  merged with vision 
    ); // TODO we set initialPoseMeters from the selected Auto

    //& SUBSYSTEM DECLARATION
    private final Vision sysVision = new Vision(//
            () -> drivetrain.getPigeon2().getRotation2d().getDegrees(),
            matrix -> poseEstimator.setVisionMeasurementStdDevs(matrix),
            (pose, time) -> poseEstimator.addVisionMeasurement(//* vision overwritten here
                    pose, time)//
    );
    private final Elevator sysElevator = new Elevator();
    private final Pivot sysPivot = new Pivot();
    // private final Climber m_climber = new Climber();
    private final CoralHandler sysCoralHandler = new CoralHandler();
    private final AlgaeHandler sysAlgaeHandler = new AlgaeHandler();
    private final AlgaeFloorIntake sysAlgaeFloorIntake =
            new AlgaeFloorIntake(new AlgaeFloorIntakeArm(), new AlgaeFloorIntakeRoller());
    private final CANRanges sysCANRanges = new CANRanges();

    private final ControlBoard LAPTOP = new ControlBoard(this.sysElevator, this.sysPivot,
            this.sysCoralHandler, this.sysAlgaeHandler, this.sysAlgaeFloorIntake);

    private final PowerDistribution pdh = new PowerDistribution(30, ModuleType.kRev); // rev PD

    // other stuff
    private final boolean allowMusic = false;
    private final boolean enablePIDTuningMode = false;
    private final SendablePID elevatorUpPID =
            new SendablePID("Elevator.Up", (float) Settings.Elevator.kP_UP,
                    (float) Settings.Elevator.kI_UP, (float) Settings.Elevator.kD_UP, 20);
    private final SendablePID elevatorDownPID =
            new SendablePID("Elevator.Down", (float) Settings.Elevator.kP_DOWN,
                    (float) Settings.Elevator.kI_DOWN, (float) Settings.Elevator.kD_DOWN, 20);
    private final SendablePID pivotPID = new SendablePID("Pivot", (float) Settings.Pivot.kP,
            (float) Settings.Pivot.kI, (float) Settings.Pivot.kD, 0.3f);

    public RobotContainer() {
        // auto
        this.configureNamedCommands();
        this.autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        // teleop
        this.configureBindings();

        // misc
        this.enableSwitchChannelPDH();
        this.drivetrain.setPoseUpdater(//
                (rotation, swerveModulePosition) -> this.poseEstimator
                        .updateWithTime(Timer.getFPGATimestamp(), rotation, swerveModulePosition));
        drivetrain.registerTelemetry(logger::telemeterize);
        SmartDashboard.putData("Field", this.field);
        this.setupPIDTuning();
    }

    private void configureBindings() {
        this.configureDriverBindings();
        this.configureOperatorBindings();
        this.configureDashboardBindings();
    }

    private SwerveRequest.FieldCentric getFieldCentricDrive() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        return this.drive//
                .withVelocityX( // Drive forward with positive Y (forward)
                        Math.pow(MathUtil.applyDeadband(this.DRIVER.getLeftY(), 0.01), 3)//
                                * this.MaxSpeed * this.drivetrain.speedMultiplier())
                .withVelocityY( // Drive left with positive X (left)
                        Math.pow(MathUtil.applyDeadband(this.DRIVER.getLeftX(), 0.01), 3)//
                                * this.MaxSpeed * this.drivetrain.speedMultiplier())
                .withRotationalRate( // Drive counterclockwise with negative X (left)
                        MathUtil.applyDeadband(-this.DRIVER.getRightX(), 0.01)//
                                * this.MaxAngularRate * this.drivetrain.speedMultiplier());
    }

    private void configureDriverBindings() {
        /* Configure Drive */
        this.drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(this::getFieldCentricDrive).withName("drivetrainDefault"));

        this.DRIVER.a().whileTrue(this.drivetrain.applyRequest(() -> this.brake)
                .withName("drivetrain brake (DRIVER.a)"));
        this.DRIVER.b()
                .whileTrue(this.drivetrain
                        .applyRequest(() -> this.point.withModuleDirection(
                                new Rotation2d(this.DRIVER.getLeftY(), this.DRIVER.getLeftX())))
                        .withName("drivetrain point at direction (DRIVER.b)"));

        // slightly unsafe
        // this.DRIVER.x().onTrue(Commands.runOnce(() -> {
        //     this.resetRelativeEncoders();
        // }, this.sysElevator, this.sysPivot));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.

        //&& DYNAMIC TEST WITH SYSID

        // && DRIVER BACK AND Y 
        // * Starts SYSID dynamic directions

        // ! CHANGE AFTER TESTING CHANGE AFTER CHANGE CHANGE CHANGE
        // DRIVER.povDown().and(DRIVER.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));

        // && DRIVER BACK AND X
        // * Starts SYSID dynamic directions
        // DRIVER.povDown().and(DRIVER.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));

        // && START BACK AND Y 
        // * Starts SYSID dynamic directions
        // DRIVER.povDown().and(DRIVER.a()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));

        // && START AND X 
        // * TOGGLES REVERSE
        // DRIVER.povDown().and(DRIVER.b()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // && LEFT BUMPER
        // * RESET FIELD CENTRIC DRIVE
        // reset the field-centric heading on left bumper press
        this.DRIVER.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric())
                .withName("reset field-centric heading (DRIVER.leftBumper)"));

        //&
        // left center button
        this.DRIVER.back().onTrue(this.drivetrain.runOnce(() -> {
            this.drivetrain.decreaseSpeedMultiplier();
        }).withName("decrease speed mult (DRIVER.back)"));
        // right center button
        this.DRIVER.start().onTrue(this.drivetrain.runOnce(() -> {
            this.drivetrain.increaseSpeedMultiplier();
        }).withName("increase speed mult (DRIVER.start)"));
        // TODO -- we can implement "♦oned" speeds using the pose estimator

        /* Configure Climb */
        // DRIVER.a().onTrue(this.m_climber.releaseClimbWinch());
        // DRIVER.rightBumper().whileTrue(this.m_climber.commandSetClimbSpeed(-1))
        //         .onFalse(this.m_climber.commandSetClimbSpeed(0));
        // DRIVER.rightTrigger().whileTrue(this.m_climber.commandSetClimbSpeed(0.5))
        //         .onFalse(this.m_climber.commandSetClimbSpeed(0));

        this.DRIVER.povDown().onTrue(this.sysAlgaeFloorIntake.setSpeeds(-0.6, 0.6))
                .onFalse(this.sysAlgaeFloorIntake.setSpeeds(0, 0));
        this.DRIVER.povUp().onTrue(this.sysAlgaeFloorIntake.setSpeeds(0.6, 0))
                .onFalse(this.sysAlgaeFloorIntake.setSpeeds(0, 0));

        this.DRIVER.povLeft().whileTrue(AutoAlign.alignToLeftReef(drivetrain, sysCANRanges));
        this.DRIVER.povRight().whileTrue(AutoAlign.alignToRightReef(drivetrain, sysCANRanges));

        this.DRIVER.rightTrigger().onTrue(this.sysAlgaeFloorIntake.setSpeeds(0, 0.6))
                .onFalse(this.sysAlgaeFloorIntake.setSpeeds(0, 0));
        this.DRIVER.leftTrigger().onTrue(this.sysAlgaeFloorIntake.setSpeeds(0, -0.6))
                .onFalse(this.sysAlgaeFloorIntake.setSpeeds(0, 0));

        if (this.allowMusic) {
            this.DRIVER.y().whileTrue(new PlayMusic("c-maj-test.chrp", this.drivetrain));
        }
    }

    private void configureOperatorBindings() {
        // TODO wrap this in
        // if (this.enablePIDTuningMode) {}
        /* Configure Elevator */
        if (this.enablePIDTuningMode) {
            Command elevatorDefault = this.sysElevator.runOnce(//
                    () -> this.sysElevator.setSpeed(
                            MathUtil.applyDeadband((-1. * OPERATOR.getLeftY()), 0.1) / 2.25));
            elevatorDefault.addRequirements(sysElevator);
            elevatorDefault.setName("elevatorDefault");
            this.sysElevator.setDefaultCommand(elevatorDefault);
            // on the controller: up == -1, down == 1

            /* Configure Pivot */
            Command pivotDefault = this.sysPivot.runOnce(//
                    () -> this.sysPivot.setSpeed(
                            MathUtil.applyDeadband((-1. * this.OPERATOR.getRightY()), 0.1) / 2.25));
            pivotDefault.addRequirements(sysPivot);
            pivotDefault.setName("pivotDefault");
            this.sysPivot.setDefaultCommand(pivotDefault);
            // on the controller: up == -1, down == 1
        } else {
            Command elevatorDefault = this.sysElevator.runOnce(//
                    () -> this.sysElevator.setSpeed(
                            MathUtil.applyDeadband((-1. * OPERATOR.getLeftY()), 0.1) / 2.));
            elevatorDefault.addRequirements(sysElevator);
            elevatorDefault.setName("elevatorDefault");
            this.sysElevator.setDefaultCommand(elevatorDefault);
            // on the controller: up == -1, down == 1

            /* Configure Pivot */
            Command pivotDefault = this.sysPivot.runOnce(//
                    () -> this.sysPivot.setSpeed(
                            MathUtil.applyDeadband((-1. * this.OPERATOR.getRightY()), 0.1) / 2.));
            pivotDefault.addRequirements(sysPivot);
            pivotDefault.setName("pivotDefault");
            this.sysPivot.setDefaultCommand(pivotDefault);
            // on the controller: up == -1, down == 1
        }

        /* Configure joint Elevator/Pivot positioning */
        if (this.enablePIDTuningMode) {
            // tuning mode:
            // a == re-flash elevator motors (UP and DOWN) and go to UP setpoint
            // b == re-flash pivot motor and go to setpoint
            // todo other subsystems
            this.OPERATOR.a().whileTrue(//
                    this.sysElevator.runOnce(() -> {
                        this.sysElevator.configureMotors(this.elevatorUpPID.P(),
                                this.elevatorUpPID.I(), this.elevatorUpPID.D(),
                                this.elevatorDownPID.P(), this.elevatorDownPID.I(),
                                this.elevatorDownPID.D());
                    }).andThen(this.sysElevator.startEnd(
                            () -> this.sysElevator.setReference(this.elevatorUpPID.SetPoint()),
                            () -> this.sysElevator.stopElevator()))
                            .withName("Elevator PID or L1 (OPERATOR.a)"))
                    .onFalse(PositionerFactory
                            .Stop(this.sysElevator, this.sysPivot, this.sysCoralHandler,
                                    this.sysAlgaeHandler, this.sysAlgaeFloorIntake)
                            .withName("stop all (OPERATOR.a off)"));

            this.OPERATOR.b().whileTrue(//
                    this.sysPivot.runOnce(() -> {
                        this.sysPivot.configureMotors(this.pivotPID.P(), this.pivotPID.I(),
                                this.pivotPID.D());
                    }).andThen(this.sysPivot.startEnd(
                            () -> this.sysPivot.setReference(this.pivotPID.SetPoint()),
                            () -> this.sysPivot.stopPivot()))
                            .withName("Pivot PID or L2 (OPERATOR.b)"))
                    .onFalse(PositionerFactory
                            .Stop(this.sysElevator, this.sysPivot, this.sysCoralHandler,
                                    this.sysAlgaeHandler, this.sysAlgaeFloorIntake)
                            .withName("stop all (OPERATOR.b off)"));
        } else {
            this.OPERATOR.a()
                    .onTrue(PositionerFactory
                            .L1(this.sysElevator, this.sysPivot, this.sysCoralHandler,
                                    this.sysAlgaeHandler, this.sysAlgaeFloorIntake)
                            .withName("L1 (OPERATOR.a)"));
            this.OPERATOR.b()
                    .onTrue(PositionerFactory
                            .L2(this.sysElevator, this.sysPivot, this.sysCoralHandler,
                                    this.sysAlgaeHandler, this.sysAlgaeFloorIntake)
                            .withName("L2 (OPERATOR.b)"));
            this.OPERATOR.x()
                    .onTrue(PositionerFactory
                            .L3(this.sysElevator, this.sysPivot, this.sysCoralHandler,
                                    this.sysAlgaeHandler, this.sysAlgaeFloorIntake)
                            .withName("L3 (OPERATOR.x)"));
            this.OPERATOR.y()
                    .onTrue(PositionerFactory
                            .L4(this.sysElevator, this.sysPivot, this.sysCoralHandler,
                                    this.sysAlgaeHandler, this.sysAlgaeFloorIntake)
                            .withName("L4 (OPERATOR.y)"));
        }

        /* Configure CoralHandler */

        //& OPERATOR LEFT BUMPER
        //* Intakes coral
        this.OPERATOR.leftBumper()
                .onTrue(this.sysCoralHandler.intake()
                        .withName("coral intake (OPERATOR.leftBumper)"))
                .onFalse(this.sysCoralHandler.contain()
                        .withName("coral contain (OPERATOR.leftBumper off)"));

        //& OPERATOR LEFT TRIGGER
        //* Spits out coral 
        this.OPERATOR.leftTrigger()
                .onTrue(this.sysCoralHandler.spitItOut()
                        .withName("coral outtake (OPERATOR.leftTrigger)"))
                .onFalse(this.sysCoralHandler.stop()
                        .withName("coral stop (OPERATOR.leftTrigger off)"));

        /* Configure AlgaeHandler */

        //& RIGHT BUMPERS
        //* ALGAE HANDLER, N/A */
        this.OPERATOR.rightBumper()
                .onTrue(this.sysAlgaeHandler.rotateCW_Intake()
                        .withName("algae clockwise (OPERATOR.rightBumper)"))
                .onFalse(this.sysAlgaeHandler.contain()
                        .withName("algae contain (OPERATOR.rightBumper off)"));
        this.OPERATOR.rightTrigger()
                .onTrue(this.sysAlgaeHandler.rotateCCW_Outtake()
                        .withName("algae counterclockwise (OPERATOR.rightTrigger)"))
                .onFalse(this.sysAlgaeHandler.stop()
                        .withName("algae stop (OPERATOR.rightTrigger off)"));

        /* Algae Intake */

        // on d-pad down, zero the current elevator position
        // OPERATOR.povDown().onTrue(//
        //         this.m_elevator.runOnce(() -> this.m_elevator.resetElevatorPosition()));

        // on d-pad up, tell the elevator and pivot to use _speed_ control
        // with the joysticks, instead of PID position control
        this.OPERATOR.start().onTrue(//
                Commands.parallel(//
                        this.sysElevator.runOnce(() -> this.sysElevator.useSpeed()), //
                        this.sysPivot.runOnce(() -> this.sysPivot.useSpeed())));

        // for testing the spoiler arm position PID
        // this.OPERATOR.povUp().onTrue(this.sysAlgaeFloorIntake.runOnce(() -> {
        //     this.sysAlgaeFloorIntake.setArmAndRoller(0.,0.);
        // }));

        this.OPERATOR.povLeft()
                .onTrue(PositionerFactory
                        .AlgaeL2(this.sysElevator, this.sysPivot, this.sysCoralHandler,
                                this.sysAlgaeHandler, this.sysAlgaeFloorIntake)
                        .withName("Algae.L2 (OPERATOR.left)"));
        this.OPERATOR.povRight()
                .onTrue(PositionerFactory
                        .AlgaeL3(this.sysElevator, this.sysPivot, this.sysCoralHandler,
                                this.sysAlgaeHandler, this.sysAlgaeFloorIntake)
                        .withName("Algae.L3 (OPERATOR.right)"));
        this.OPERATOR.povUp()
                .onTrue(PositionerFactory
                        .Barge(this.sysElevator, this.sysPivot, this.sysCoralHandler,
                                this.sysAlgaeHandler, this.sysAlgaeFloorIntake)
                        .withName("Algae.Barge (OPERATOR.up)"));
        this.OPERATOR.povDown()
                .onTrue(PositionerFactory
                        .AlgaeGround(this.sysElevator, this.sysPivot, this.sysCoralHandler,
                                this.sysAlgaeHandler, this.sysAlgaeFloorIntake)
                        .withName("Algae.Ground (OPERATOR.down)"));
    }

    private void configureDashboardBindings() {
        SmartDashboard.putData("control-states", this.LAPTOP);
    }

    public Command getAutonomousCommand() {
        this.poseEstimator.resetPose(AutoBuilder.getCurrentPose());
        // return the autoChooser's selected Auto
        return this.autoChooser.getSelected();
    }

    private void configureNamedCommands() {
        // && POSITIONS
        // TODO -- what happens if we use the same command instance twice?
        NamedCommands.registerCommand("L1 Position",
                PositionerFactory.L1(this.sysElevator, this.sysPivot, this.sysCoralHandler,
                        this.sysAlgaeHandler, this.sysAlgaeFloorIntake));
        NamedCommands.registerCommand("L2 Position",
                PositionerFactory.L2(this.sysElevator, this.sysPivot, this.sysCoralHandler,
                        this.sysAlgaeHandler, this.sysAlgaeFloorIntake));
        NamedCommands.registerCommand("L3 Position",
                PositionerFactory.L3(this.sysElevator, this.sysPivot, this.sysCoralHandler,
                        this.sysAlgaeHandler, this.sysAlgaeFloorIntake));
        NamedCommands.registerCommand("L4 Position",
                PositionerFactory.L4(this.sysElevator, this.sysPivot, this.sysCoralHandler,
                        this.sysAlgaeHandler, this.sysAlgaeFloorIntake));
        NamedCommands.registerCommand("Feed",
                PositionerFactory.Feed(this.sysElevator, this.sysPivot, this.sysCoralHandler,
                        this.sysAlgaeHandler, this.sysAlgaeFloorIntake));

        NamedCommands.registerCommand("Run Coral Intake", this.sysCoralHandler.intake());
        NamedCommands.registerCommand("Run Coral Outake", this.sysCoralHandler.spitItOut());
        NamedCommands.registerCommand("Stop Coral", this.sysCoralHandler.stop());

        NamedCommands.registerCommand("♦ Reef Right",
                AutoAlign.alignToRightReef(this.drivetrain, this.sysCANRanges));

        // NamedCommands.registerCommand("Climb Inwards", m_climber.commandSetClimbSpeed(-1));
        // NamedCommands.registerCommand("Climb Inwards", m_climber.commandSetClimbSpeed(1));
        // NamedCommands.registerCommand("Stop Climbter", m_climber.commandSetClimbSpeed(1));
    }

    public void updateField() {
        this.field.setRobotPose(this.poseEstimator.getEstimatedPosition());
        this.field.getObject("swervePose").setPose(this.drivetrain.getPose());
        // can also set trajectories
        // https://docs.wpilib.org/en/stable/docs/software/dashboards/glass/field2d-widget.html#sending-trajectories-to-field2d
    }

    public void zero() {
        this.sysElevator.stopElevator();
        this.sysPivot.stopPivot();
    }

    public void resetRelativeEncoders() {
        this.sysElevator.resetElevatorPosition();
        this.sysAlgaeFloorIntake.resetArmPosition();
    }


    private void setupPIDTuning() {
        if (!this.enablePIDTuningMode) {
            return;
        }
        SmartDashboard.putBoolean(SendablePID.prefix + "/Tuning Mode", false);
        SmartDashboard.putData(SendablePID.prefix, this.elevatorUpPID);
        SmartDashboard.putData(SendablePID.prefix, this.elevatorDownPID);
        SmartDashboard.putData(SendablePID.prefix, this.pivotPID);
    }

    private boolean isTuningMode() {
        if (!this.enablePIDTuningMode) {
            return false;
        }
        // System.out.printf("tuning mode: %b\n", SmartDashboard.getBoolean(SendablePID.prefix + "/Tuning Mode", false));
        return SmartDashboard.getBoolean(SendablePID.prefix + "/Tuning Mode", false);
    }

    private void enableSwitchChannelPDH() {
        this.pdh.setSwitchableChannel(true);
    }
}
