// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Commands.CoralPositionFactory;
import frc.robot.Constants.Settings;
import frc.robot.subsystems.Pivot;
import frc.robot.generated.TunerConstants;
import frc.robot.sendables.ControlBoard;
import frc.robot.sendables.SendablePID;
import frc.robot.subsystems.AlgaeFloorIntake;
import frc.robot.subsystems.AlgaeHandler;
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

    private final ControlBoard LAPTOP = new ControlBoard(sysElevator, sysPivot);

    // other stuff
    private final Orchestra orchestra = new Orchestra();

    private final boolean turnOffTuning = false;
    private final SendablePID elevatorUpPID =
            new SendablePID("Elevator.Up", (float) Settings.Elevator.kP_UP,
                    (float) Settings.Elevator.kI_UP, (float) Settings.Elevator.kD_UP, 20);
    private final SendablePID elevatorDownPID =
            new SendablePID("Elevator.Down", (float) Settings.Elevator.kP_DOWN,
                    (float) Settings.Elevator.kI_DOWN, (float) Settings.Elevator.kD_DOWN, 20);
    private final SendablePID pivotPID = new SendablePID("Pivot", (float) Settings.Pivot.kP,
            (float) Settings.Pivot.kI, (float) Settings.Pivot.kD, 100);

    public RobotContainer() {
        // auto
        this.configureNamedCommands();
        this.autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        // teleop
        this.configureBindings();

        // misc
        this.drivetrain.setPoseUpdater(//
                (rotation, swerveModulePosition) -> this.poseEstimator
                        .updateWithTime(Timer.getFPGATimestamp(), rotation, swerveModulePosition));
        drivetrain.registerTelemetry(logger::telemeterize);
        SmartDashboard.putData("Field", this.field);
        this.setupPIDTuning();
        if (true) { // todo ignore
            this.configureMusic();
        }
    }

    private void configureMusic() {
        for (SwerveModule<TalonFX, TalonFX, CANcoder> module : this.drivetrain.getModules()) {
            orchestra.addInstrument(module.getDriveMotor());
            orchestra.addInstrument(module.getSteerMotor());
        }
        var status = orchestra.loadMusic(Filesystem.getDeployDirectory() + "/chrp/output.chrp");
        if (!status.isOK()) {
            // log error
            DriverStation.reportError(status.toString(), true);
        } else {
            System.out.println(orchestra.play());
        }
        System.out.println("###############################");
        System.out.println(orchestra.isPlaying());
    }

    private void configureBindings() {
        this.configureDriverBindings();
        this.configureOperatorBindings();
        this.configureDashboardBindings();
    }

    private SwerveRequest.FieldCentric getFieldCentricDrive() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        return drive//
                .withVelocityX( // Drive forward with positive Y (forward)
                        MathUtil.applyDeadband(DRIVER.getLeftY(), 0.1)//
                                * MaxSpeed * drivetrain.speedMultiplier())
                .withVelocityY( // Drive left with positive X (left)
                        MathUtil.applyDeadband(DRIVER.getLeftX(), 0.1)//
                                * MaxSpeed * drivetrain.speedMultiplier())
                .withRotationalRate( // Drive counterclockwise with negative X (left)
                        MathUtil.applyDeadband(-DRIVER.getRightX(), 0.1)//
                                * MaxAngularRate * drivetrain.speedMultiplier());
    }

    private void configureDriverBindings() {
        /* Configure Drive */
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(this::getFieldCentricDrive).withName("drivetrainDefault"));

        DRIVER.a().whileTrue(drivetrain.applyRequest(() -> brake));
        DRIVER.b().whileTrue(drivetrain.applyRequest(() -> point
                .withModuleDirection(new Rotation2d(DRIVER.getLeftY(), DRIVER.getLeftX()))));

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
        DRIVER.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //&
        // left center button
        DRIVER.back().onTrue(this.drivetrain.runOnce(() -> {
            this.drivetrain.decreaseSpeedMultiplier();
        }));
        // right center button
        DRIVER.start().onTrue(this.drivetrain.runOnce(() -> {
            this.drivetrain.increaseSpeedMultiplier();
        }));
        // TODO -- we can implement "zoned" speeds using the pose estimator

        /* Configure Climb */
        // DRIVER.a().onTrue(this.m_climber.releaseClimbWinch());
        // DRIVER.rightBumper().whileTrue(this.m_climber.commandSetClimbSpeed(-1))
        //         .onFalse(this.m_climber.commandSetClimbSpeed(0));
        // DRIVER.rightTrigger().whileTrue(this.m_climber.commandSetClimbSpeed(0.5))
        //         .onFalse(this.m_climber.commandSetClimbSpeed(0));
    }

    public void zero() {
        this.sysElevator.stopElevator();
        this.sysPivot.stopPivot();
    }

    private void configureOperatorBindings() {
        /* Configure Elevator */
        Command elevatorDefault = Commands.either(//
                Commands.none(), //
                this.sysElevator.runOnce(//
                        () -> this.sysElevator.setSpeed(
                                MathUtil.applyDeadband((-1. * OPERATOR.getLeftY()), 0.1) / 2.25)), //
                this::isTuningMode);
        elevatorDefault.addRequirements(sysElevator);
        elevatorDefault.setName("elevatorDefault");
        this.sysElevator.setDefaultCommand(elevatorDefault);
        // on the controller: up == -1, down == 1

        /* Configure Pivot */
        Command pivotDefault = Commands.either(//
                Commands.none(), //
                this.sysPivot.runOnce(//
                        () -> this.sysPivot.setSpeed(
                                MathUtil.applyDeadband((-1. * OPERATOR.getRightY()), 0.1) / 2.25)), //
                this::isTuningMode);
        pivotDefault.addRequirements(sysPivot);
        pivotDefault.setName("pivotDefault");
        this.sysPivot.setDefaultCommand(pivotDefault);
        // on the controller: up == -1, down == 1

        /* Configure joint Elevator/Pivot positioning */
        if (this.turnOffTuning) {
            this.OPERATOR.a().onTrue(CoralPositionFactory.L1(this.sysElevator, this.sysPivot))
                    .onFalse(CoralPositionFactory.Stop(this.sysElevator, this.sysPivot));
            this.OPERATOR.b().onTrue(CoralPositionFactory.L2(this.sysElevator, this.sysPivot))
                    .onFalse(CoralPositionFactory.Stop(this.sysElevator, this.sysPivot));
            this.OPERATOR.x().onTrue(CoralPositionFactory.L3(this.sysElevator, this.sysPivot))
                    .onFalse(CoralPositionFactory.Stop(this.sysElevator, this.sysPivot));
            this.OPERATOR.y().onTrue(CoralPositionFactory.L4(this.sysElevator, this.sysPivot))
                    .onFalse(CoralPositionFactory.Stop(this.sysElevator, this.sysPivot));
        } else {
            // tuning mode:
            // a == re-flash elevator motors (UP and DOWN) and go to UP setpoint
            // b == re-flash pivot motor and go to setpoint
            // todo other subsystems
            this.OPERATOR.a().onTrue(//
                    Commands.either(//
                            this.sysElevator.runOnce(() -> {
                                this.sysElevator.configureMotors(this.elevatorUpPID.P(),
                                        this.elevatorUpPID.I(), this.elevatorUpPID.D(),
                                        this.elevatorDownPID.P(), this.elevatorDownPID.I(),
                                        this.elevatorDownPID.D());
                            }).andThen(this.sysElevator.startEnd(
                                    () -> this.sysElevator
                                            .setReference(this.elevatorUpPID.SetPoint()),
                                    () -> this.sysElevator.stopElevator())), //
                            CoralPositionFactory.L1(this.sysElevator, this.sysPivot), //
                            this::isTuningMode).withName("OPERATOR.a().onTrue"))
                    .onFalse(CoralPositionFactory.Stop(this.sysElevator, this.sysPivot));

            this.OPERATOR.b().onTrue(//
                    Commands.either(//
                            this.sysPivot.runOnce(() -> {
                                this.sysPivot.configureMotors(this.pivotPID.P(), this.pivotPID.I(),
                                        this.pivotPID.D());
                            }).andThen(this.sysPivot.startEnd(
                                    () -> this.sysPivot.setReference(this.pivotPID.SetPoint()),
                                    () -> this.sysPivot.stopPivot())), //
                            CoralPositionFactory.L2(this.sysElevator, this.sysPivot), //
                            this::isTuningMode).withName("OPERATOR.b().onTrue"))
                    .onFalse(CoralPositionFactory.Stop(this.sysElevator, this.sysPivot));
        }

        /* Configure CoralHandler */

        //& OPERATOR LEFT BUMPER
        //* Intakes coral
        OPERATOR.leftBumper().onTrue(this.sysCoralHandler.intake())
                .onFalse(this.sysCoralHandler.contain());

        //& OPERATOR LEFT TRIGGER
        //* Spits out coral 
        OPERATOR.leftTrigger().onTrue(this.sysCoralHandler.spitItOut())
                .onFalse(this.sysCoralHandler.stop());

        /* Configure AlgaeHandler */

        //& RIGHT BUMPERS
        //* ALGAE HANDLER, N/A */
        OPERATOR.rightBumper().onTrue(this.sysAlgaeHandler.rotateCW())
                .onFalse(this.sysAlgaeHandler.stop());
        OPERATOR.rightTrigger().onTrue(this.sysAlgaeHandler.rotateCCW())
                .onFalse(this.sysAlgaeHandler.stop());

        /* Algae Intake */

        // on d-pad down, zero the current elevator position
        // OPERATOR.povDown().onTrue(//
        //         this.m_elevator.runOnce(() -> this.m_elevator.resetElevatorPosition()));

        // on d-pad up, tell the elevator and pivot to use _speed_ control
        // with the joysticks, instead of PID position control
        OPERATOR.povUp().onTrue(//
                Commands.parallel(//
                        this.sysElevator.runOnce(() -> this.sysElevator.useSpeed()), //
                        this.sysPivot.runOnce(() -> this.sysPivot.useSpeed())));
        // OPERATOR.povDown().onTrue(CoralPositionFactory.Feed(m_elevator, m_pivot));

        OPERATOR.povLeft().onTrue(this.sysAlgaeFloorIntake.setStuff(1, 0))
                .onFalse(this.sysAlgaeFloorIntake.setStuff(0, 0));
        OPERATOR.povRight().onTrue(this.sysAlgaeFloorIntake.setStuff(-1, 0))
                .onFalse(this.sysAlgaeFloorIntake.setStuff(0, 0));
        OPERATOR.povDown().onTrue(this.sysAlgaeFloorIntake.setStuff(0, 1))
                .onFalse(this.sysAlgaeFloorIntake.setStuff(0, 0));
    }

    private void configureDashboardBindings() {
        SmartDashboard.putData("control-states", this.LAPTOP);
    }

    public Command getAutonomousCommand() {
        this.poseEstimator.resetPose(AutoBuilder.getCurrentPose());
        // return the autoChooser's selected Auto
        return this.autoChooser.getSelected();

        // ! DEBUG FEEDER LATER
        // return Commands.sequence(
        //     CoralPositionFactory.L1(m_elevator, m_pivot),
        //     Commands.waitSeconds(0.2),
        //     CoralPositionFactory.Feed(m_elevator, m_pivot),
        //     Commands.waitSeconds(0.2),
        //     CoralPositionFactory.L1(m_elevator, m_pivot)
        // );
        // return Commands.sequence(
        //     CoralPositionFactory.Feed(m_elevator, m_pivot),
        //     m_coralHandler.commandIntakeCoral()
        // );

    }

    private void configureNamedCommands() {
        // && POSITIONS
        // TODO -- what happens if we use the same command instance twice?
        NamedCommands.registerCommand("L1 Position",
                CoralPositionFactory.L1(sysElevator, sysPivot));
        NamedCommands.registerCommand("L2 Position",
                CoralPositionFactory.L2(sysElevator, sysPivot));
        NamedCommands.registerCommand("L3 Position",
                CoralPositionFactory.L3(sysElevator, sysPivot));
        NamedCommands.registerCommand("L4 Position",
                CoralPositionFactory.L4(sysElevator, sysPivot));
        NamedCommands.registerCommand("Feed", CoralPositionFactory.Feed(sysElevator, sysPivot));

        // TODO THESE MUST BE TESTED
        NamedCommands.registerCommand("Run Coral Intake", sysCoralHandler.intake());
        NamedCommands.registerCommand("Run Coral Outake", sysCoralHandler.spitItOut());
        NamedCommands.registerCommand("Stop Coral Intake", sysCoralHandler.stop());

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

    private void setupPIDTuning() {
        if (this.turnOffTuning)
            return;
        SmartDashboard.putBoolean(SendablePID.prefix + "/Tuning Mode", false);
    }

    private boolean isTuningMode() {
        if (this.turnOffTuning)
            return false;
        return SmartDashboard.getBoolean(SendablePID.prefix + "/Tuning Mode", false);
    }
}
