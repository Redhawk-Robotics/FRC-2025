// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.Commands.RunToPose;
// import frc.robot.Commands.DriveToPose;
import frc.robot.Commands.PlayMusic;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Commands.PositionerFactory;
import frc.robot.Constants.Settings;
import frc.robot.subsystems.Pivot;
import frc.robot.sendables.ControlBoard;
import frc.robot.sendables.SendablePID;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.AlgaeRoller;
import frc.robot.subsystems.AlgaeHandler;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.Telemetry;
import frc.robot.subsystems.swerve.TunerConstants;
import frc.robot.subsystems.CoralHandler;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.elevator.Elevator;

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

    //& SUBSYSTEM DECLARATION
    private final Vision sysVision = new Vision(//
            () -> this.drivetrain.getPigeon2().getRotation2d().getDegrees(),
            matrix -> this.drivetrain.setVisionMeasurementStdDevs(matrix),
            (pose, time) -> this.drivetrain.addVisionMeasurement(//* vision overwritten here
                    pose, time)//
    ); // TODO verify Vision provides working MegaTag2 localization updates
    private final Elevator sysElevator = Elevator.getInstance();
    private final Pivot sysPivot = new Pivot();
    private final AlgaeArm sysSpoiler = new AlgaeArm();
    private final CoralHandler sysCoralHandler = new CoralHandler();
    private final AlgaeHandler sysAlgaeHandler = new AlgaeHandler();
    private final AlgaeRoller sysRoller = new AlgaeRoller();
    // private final CANRanges sysCANRanges = new CANRanges(); // TODO re-enable (if using)

    private final ControlBoard LAPTOP =
            new ControlBoard(this.sysElevator, this.sysPivot, this.sysSpoiler);

    private final PowerDistribution pdh = new PowerDistribution(30, ModuleType.kRev); // rev PD

    // other stuff
    private final boolean allowMusic = false;
    private final boolean enablePIDTuningMode = false;
    private final SendablePID elevatorUpPID =
            new SendablePID("Elevator.Up", (float) Settings.Elevator.kP,
                    (float) Settings.Elevator.kI, (float) Settings.Elevator.kD, 4);
    private final SendablePID pivotPID = new SendablePID("Pivot", (float) Settings.Pivot.kP,
            (float) Settings.Pivot.kI, (float) Settings.Pivot.kD, 0.3f);

    public RobotContainer() {
        // auto
        this.configureNamedCommands();
        this.autoChooser = AutoBuilder.buildAutoChooser("I LOVE NYC");
        this.configureAutoChooser();

        // teleop
        this.configureBindings();

        // misc
        this.enableSwitchChannelPDH();
        this.drivetrain.registerTelemetry(logger::telemeterize);
        this.setupPIDTuning();
    }

    private void configureBindings() {
        this.configureDriverBindings();
        this.configureOperatorBindings();
        // this.configureDashboardBindings();
    }

    private SwerveRequest.FieldCentric getFieldCentricDrive() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        return this.drive//
                .withVelocityX( // Drive forward with positive Y (forward)
                        Math.pow(MathUtil.applyDeadband(this.DRIVER.getLeftY(), 0.05), 3)//
                                * this.MaxSpeed * this.drivetrain.speedMultiplier())
                .withVelocityY( // Drive left with positive X (left)
                        Math.pow(MathUtil.applyDeadband(this.DRIVER.getLeftX(), 0.05), 3)//
                                * this.MaxSpeed * this.drivetrain.speedMultiplier())
                .withRotationalRate( // Drive counterclockwise with negative X (left)
                        MathUtil.applyDeadband(-this.DRIVER.getRightX(), 0.05)//
                                * this.MaxAngularRate * this.drivetrain.speedMultiplier());
    }

    private void configureDriverBindings() {
        /* Configure Drive */
        this.drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(this::getFieldCentricDrive).withName("drivetrainDefault"));

        this.DRIVER.x().whileTrue(this.drivetrain.applyRequest(() -> this.brake)
                .withName("drivetrain brake (DRIVER.x)"));
        this.DRIVER.y()
                .whileTrue(this.drivetrain
                        .applyRequest(() -> this.point.withModuleDirection(
                                new Rotation2d(this.DRIVER.getLeftY(), this.DRIVER.getLeftX())))
                        .withName("drivetrain point at direction (DRIVER.y)"));

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

        // DRIVER.rightBumper().onTrue(new RunToPose(drivetrain, sysVision)).onFalse(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));

        //& 
        // TODO -- if this is too cumbersome, we can move these to the OPERATOR
        //&
        // left center button
        this.DRIVER.back().onTrue(this.drivetrain.runOnce(() -> {
            this.drivetrain.decreaseSpeedMultiplier();
        }).withName("decrease speed mult (DRIVER.back)"));
        // right center button
        this.DRIVER.start().onTrue(this.drivetrain.runOnce(() -> {
            this.drivetrain.increaseSpeedMultiplier();
        }).withName("increase speed mult (DRIVER.start)"));
        // TODO -- we can implement "zoned" speeds using the pose estimator

        this.DRIVER.povDown()
                .onTrue(this.sysSpoiler.runOnce(() -> this.sysSpoiler.setSpeed(0.6))
                        .alongWith(this.sysRoller.runOnce(() -> this.sysRoller.setSpeed(0.6))))
                .onFalse(this.sysSpoiler.runOnce(() -> this.sysSpoiler.setSpeed(0))
                        .alongWith(this.sysRoller.runOnce(() -> this.sysRoller.setSpeed(0))));
        this.DRIVER.povUp()
                .onTrue(this.sysSpoiler.runOnce(() -> this.sysSpoiler.setSpeed(-0.6))
                        .alongWith(this.sysRoller.runOnce(() -> this.sysRoller.setSpeed(0))))
                .onFalse(this.sysSpoiler.runOnce(() -> this.sysSpoiler.setSpeed(0))
                        .alongWith(this.sysRoller.runOnce(() -> this.sysRoller.setSpeed(0))));

        // TODO re-enable if using
        // this.DRIVER.povLeft().whileTrue(AutoAlign.alignToLeftReef(drivetrain, sysCANRanges));
        // this.DRIVER.povRight().whileTrue(AutoAlign.alignToRightReef(drivetrain, sysCANRanges));

        this.DRIVER.rightTrigger()
                .onTrue(this.sysSpoiler.runOnce(() -> this.sysSpoiler.setSpeed(0))
                        .alongWith(this.sysRoller.runOnce(() -> this.sysRoller.setSpeed(0.6))))
                .onFalse(this.sysSpoiler.runOnce(() -> this.sysSpoiler.setSpeed(0))
                        .alongWith(this.sysRoller.runOnce(() -> this.sysRoller.setSpeed(0))));
        this.DRIVER.leftTrigger()
                .onTrue(this.sysSpoiler.runOnce(() -> this.sysSpoiler.setSpeed(0))
                        .alongWith(this.sysRoller.runOnce(() -> this.sysRoller.setSpeed(-0.6))))
                .onFalse(this.sysSpoiler.runOnce(() -> this.sysSpoiler.setSpeed(0))
                        .alongWith(this.sysRoller.runOnce(() -> this.sysRoller.setSpeed(0))));

        this.DRIVER.a()
                .onTrue(PositionerFactory
                        .AlgaeGround(this.sysElevator, this.sysPivot, this.sysSpoiler)
                        .withName("AlgaeGround (DRIVER.a)"));
        this.DRIVER.b()
                .onTrue(PositionerFactory
                        .AlgaeTransfer(this.sysElevator, this.sysPivot, this.sysSpoiler)
                        .withName("AlgaeTransfer (DRIVER.b)"));

        if (this.allowMusic) {
            this.DRIVER.y().whileTrue(new PlayMusic("c-maj-test.chrp", this.drivetrain));
        }
    }

    private void configureOperatorBindings() {
        /* Configure joint Elevator/Pivot positioning */
        if (this.enablePIDTuningMode) {
            // tuning mode:
            // a == re-flash elevator motors (UP and DOWN) and go to UP setpoint
            // b == re-flash pivot motor and go to setpoint
            // todo other subsystems
            this.OPERATOR.a().whileTrue(//
                    this.sysElevator.runOnce(() -> {
                        // TODO add kG and kS
                        this.sysElevator.configureMotors(0, 0, this.elevatorUpPID.P(),
                                this.elevatorUpPID.I(), this.elevatorUpPID.D());
                    }).andThen(this.sysElevator.startEnd(
                            () -> this.sysElevator.useControlMode(Elevator.Mode.kPosition,
                                    this.elevatorUpPID.SetPoint(), this.elevatorUpPID::SetPoint),
                            () -> this.sysElevator.stopElevator()))
                            .withName("Elevator PID (OPERATOR.a)"))
                    .onFalse(
                            PositionerFactory.Stop(this.sysElevator, this.sysPivot, this.sysSpoiler)
                                    .withName("stop all (OPERATOR.a off)"));

            this.OPERATOR.b().whileTrue(//
                    this.sysPivot.runOnce(() -> {
                        this.sysPivot.configureMotors(this.pivotPID.P(), this.pivotPID.I(),
                                this.pivotPID.D());
                    }).andThen(this.sysPivot.startEnd(
                            () -> this.sysPivot.useControlMode(Pivot.Mode.kPosition,
                                    this.pivotPID.SetPoint(), this.pivotPID::SetPoint),
                            () -> this.sysPivot.stopPivot())).withName("Pivot PID (OPERATOR.b)"))
                    .onFalse(
                            PositionerFactory.Stop(this.sysElevator, this.sysPivot, this.sysSpoiler)
                                    .withName("stop all (OPERATOR.b off)"));
        } else {
            this.OPERATOR.a()
                    .onTrue(PositionerFactory.L1(this.sysElevator, this.sysPivot, this.sysSpoiler)
                            .withName("L1 (OPERATOR.a)"));
            this.OPERATOR.b()
                    .onTrue(PositionerFactory.L2(this.sysElevator, this.sysPivot, this.sysSpoiler)
                            .withName("L2 (OPERATOR.b)"));
            this.OPERATOR.x()
                    .onTrue(PositionerFactory.L3(this.sysElevator, this.sysPivot, this.sysSpoiler)
                            .withName("L3 (OPERATOR.x)"));
            this.OPERATOR.y()
                    .onTrue(PositionerFactory.L4(this.sysElevator, this.sysPivot, this.sysSpoiler)
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

        // on start-button (right-center button), tell the elevator and pivot to use _speed_ control
        // with the joysticks, instead of PID position control
        this.OPERATOR.start().onTrue(//
                Commands.parallel(//
                        this.sysElevator.runOnce(() -> this.sysElevator.useControlMode(
                                Elevator.Mode.kManualSpeed, 0,
                                () -> MathUtil.applyDeadband((-1. * this.OPERATOR.getRightY()), 0.1)
                                        / 2.25)), //
                        this.sysPivot.runOnce(() -> this.sysPivot.useControlMode(
                                Pivot.Mode.kManualSpeed, 0,
                                () -> MathUtil.applyDeadband((-1. * this.OPERATOR.getRightY()), 0.1)
                                        / 2.))));
        // on the controller: up == -1, down == 1
        this.OPERATOR.back().onTrue(//
                Commands.parallel(//
                        this.sysElevator.runOnce(() -> this.sysElevator.useControlMode(
                                Elevator.Mode.kManualPosition,
                                this.sysElevator.getPosition().magnitude(),
                                () -> MathUtil.applyDeadband((-1. * this.OPERATOR.getRightY()), 0.1)
                                        / 2.25)), // todo tune
                        this.sysPivot.runOnce(() -> this.sysPivot.useControlMode(
                                Pivot.Mode.kManualPosition, this.sysPivot.getPosition(),
                                () -> MathUtil.applyDeadband((-1. * this.OPERATOR.getRightY()), 0.1)
                                        / 2.)))); // todo tune
        // on the controller: up == -1, down == 1

        this.OPERATOR.povLeft()
                .onTrue(PositionerFactory.AlgaeL2(this.sysElevator, this.sysPivot, this.sysSpoiler)
                        .withName("Algae.L2 (OPERATOR.left)"));
        this.OPERATOR.povRight()
                .onTrue(PositionerFactory.AlgaeL3(this.sysElevator, this.sysPivot, this.sysSpoiler)
                        .withName("Algae.L3 (OPERATOR.right)"));
        this.OPERATOR.povUp()
                .onTrue(PositionerFactory.Barge(this.sysElevator, this.sysPivot, this.sysSpoiler)
                        .withName("Algae.Barge (OPERATOR.up)"));
        this.OPERATOR.povDown()
                .onTrue(PositionerFactory
                        .AlgaeGround(this.sysElevator, this.sysPivot, this.sysSpoiler)
                        .withName("Algae.Ground (OPERATOR.down)"));
    }

    private void configureDashboardBindings() {
        SmartDashboard.putData("control-states", this.LAPTOP);
    }

    public Command getAutonomousCommand() {
        // TODO remove
        // return new DriveToPose(this.drivetrain, new Pose2d(12, 6, new Rotation2d(6)));

        // TODO uncomment
        // return the autoChooser's selected Auto
        return this.autoChooser.getSelected();
    }

    private void configureNamedCommands() {
        // && NamedCommands
        NamedCommands.registerCommand("L1 Position",
                PositionerFactory.L1(this.sysElevator, this.sysPivot, this.sysSpoiler));
        NamedCommands.registerCommand("L2 Position",
                PositionerFactory.L2(this.sysElevator, this.sysPivot, this.sysSpoiler));
        NamedCommands.registerCommand("L3 Position",
                PositionerFactory.L3(this.sysElevator, this.sysPivot, this.sysSpoiler));
        NamedCommands.registerCommand("L4 Position",
                PositionerFactory.L4(this.sysElevator, this.sysPivot, this.sysSpoiler));
        NamedCommands.registerCommand("Feed",
                PositionerFactory.Feed(this.sysElevator, this.sysPivot, this.sysSpoiler));
        NamedCommands.registerCommand("Barge",
                PositionerFactory.Barge(this.sysElevator, this.sysPivot, this.sysSpoiler));
        NamedCommands.registerCommand("Run Coral Intake", this.sysCoralHandler.intake());
        NamedCommands.registerCommand("Run Coral Contain", this.sysCoralHandler.contain());
        NamedCommands.registerCommand("Run Coral Outake", this.sysCoralHandler.spitItOut());
        NamedCommands.registerCommand("Stop Coral", this.sysCoralHandler.stop());
        // TODO add in
        NamedCommands.registerCommand("Run Algae Intake", this.sysAlgaeHandler.rotateCW_Intake());
        NamedCommands.registerCommand("Run Algae Contain", this.sysAlgaeHandler.contain());
        NamedCommands.registerCommand("Run Algae Outake", this.sysAlgaeHandler.rotateCCW_Outtake());
        NamedCommands.registerCommand("Stop Algae", this.sysAlgaeHandler.stop());
        // TODO re-enable if using
        // NamedCommands.registerCommand("Align Reef Right",
        //         AutoAlign.alignToRightReef(this.drivetrain, this.sysCANRanges));
        // NamedCommands.registerCommand("Align Reef Left",
        //         AutoAlign.alignToLeftReef(this.drivetrain, this.sysCANRanges));

        // NOTE -- we effectively cannot use these _and_ NamedCommands at the same time
        // https://pathplanner.dev/pplib-triggers.html
        // If an event-trigger schedules a command with the same requirements as
        // a command listed in a PP Auto, then the Auto is canceled.
        // This would be a more legitimate use-case for .asProxy()
        // && Event waypoint triggers
        // new EventTrigger("Run Algae Intake").onTrue(this.sysAlgaeHandler.rotateCW_Intake());
        new EventTrigger("L2 Position")
                .onTrue(PositionerFactory.L2(this.sysElevator, this.sysPivot, this.sysSpoiler));
        new EventTrigger("L4 Position")
                .onTrue(PositionerFactory.L4(this.sysElevator, this.sysPivot, this.sysSpoiler));
        new EventTrigger("Barge")
                .onTrue(PositionerFactory.Barge(this.sysElevator, this.sysPivot, this.sysSpoiler));
    }

    private void configureAutoChooser() {
        this.autoChooser.onChange(cmd -> {
            try {
                // PathPlanner does this for us at command start.
                // This helps with visualizing the pose on the Field in dashboards.
                //
                // This code just pulls the currently-selected PathPlanner Auto's starting pose
                // and resets the drivetrain to that pose.
                Pose2d pose = ((PathPlannerAuto) cmd).getStartingPose();
                var alliance = DriverStation.getAlliance();
                if (!alliance.isEmpty() && alliance.get() == Alliance.Red) {
                    pose = FlippingUtil.flipFieldPose(pose);
                }
                this.drivetrain.resetPose(pose);
            } catch (Exception e) {
                // nothing to do
            }
        });
        SmartDashboard.putData("Auto Mode", this.autoChooser);
    }

    public void zero() {
        this.sysElevator.stopElevator();
        this.sysPivot.stopPivot();
    }

    public void resetRelativeEncoders() {
        this.sysElevator.resetElevatorPosition();
        this.sysSpoiler.resetArmPosition();
    }

    private void setupPIDTuning() {
        if (!this.enablePIDTuningMode) {
            return;
        }
        SmartDashboard.putBoolean(SendablePID.prefix + "/Tuning Mode", false);
        SmartDashboard.putData(SendablePID.prefix, this.elevatorUpPID);
        // SmartDashboard.putData(SendablePID.prefix, this.elevatorDownPID);
        SmartDashboard.putData(SendablePID.prefix, this.pivotPID);
    }

    private void enableSwitchChannelPDH() {
        this.pdh.setSwitchableChannel(true);
    }
}
