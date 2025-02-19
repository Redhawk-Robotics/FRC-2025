// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.Ports;
import frc.robot.subsystems.Pivot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeHandler;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralHandler;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.CommandSwerveDrivetrain.speeds;

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

    private final CommandXboxController DRIVER = new CommandXboxController(Ports.Gamepad.DRIVER);
    private final CommandXboxController OPERATOR =
            new CommandXboxController(Ports.Gamepad.OPERATOR);

    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser("Tests");

    private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
            drivetrain.getKinematics(), drivetrain.getPigeon2().getRotation2d(),
            drivetrain.getState().ModulePositions, new Pose2d()//
    ); // TODO we set initialPoseMeters from the selected Auto

    private final Vision m_vision = new Vision(//
            () -> drivetrain.getPigeon2().getRotation2d().getDegrees(),
            matrix -> m_poseEstimator.setVisionMeasurementStdDevs(matrix),
            poseAndTime -> m_poseEstimator.addVisionMeasurement(//
                    poseAndTime.getFirst(), poseAndTime.getSecond())//
    );

    private final Elevator m_elevator = new Elevator();
    private final Pivot m_pivot = new Pivot();
    private final Climber m_climber = new Climber();
    private final CoralHandler m_coralHandler = new CoralHandler();
    private final AlgaeHandler m_algaeHandler = new AlgaeHandler();

    public RobotContainer() {
        configureBindings();

        SmartDashboard.putData("Auto Mode", autoChooser);

        this.drivetrain.setPoseUpdater(//
                t -> this.m_poseEstimator.update(t.getFirst(), t.getSecond()));
        // t -> this.m_poseEstimator.updateWithTime(Timer.getFPGATimestamp(), t.getFirst(),
        // t.getSecond())); // TODO should we use this one?
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void configureBindings() {
        this.configureDriverBindings();
        this.configureOperatorBindings();
    }

    private SwerveRequest.FieldCentric getFieldCentricDrive() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        return drive//
                .withVelocityX( // Drive forward with positive Y (forward)
                        Math.pow(DRIVER.getLeftY() * MaxSpeed * drivetrain.speedMultiplier(), 5))
                .withVelocityY( // Drive left with positive X (left)
                        Math.pow(DRIVER.getLeftX() * MaxSpeed * drivetrain.speedMultiplier(), 5))
                .withRotationalRate( // Drive counterclockwise with negative X (left)
                        Math.pow(-DRIVER.getRightX() * MaxAngularRate / 1.25, 5));
    }

    private void configureDriverBindings() {
        /* Configure Drive */
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(this::getFieldCentricDrive));

        DRIVER.a().whileTrue(drivetrain.applyRequest(() -> brake));
        DRIVER.b().whileTrue(drivetrain.applyRequest(() -> point
                .withModuleDirection(new Rotation2d(DRIVER.getLeftY(), DRIVER.getLeftX()))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        DRIVER.back().and(DRIVER.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        DRIVER.back().and(DRIVER.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        DRIVER.start().and(DRIVER.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        DRIVER.start().and(DRIVER.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        DRIVER.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // TODO -- if this is too cumbersome, we can move these to the OPERATOR
        // left center button
        DRIVER.back().onTrue(this.drivetrain.runOnce(() -> {
            this.drivetrain.decreaseSpeedMultiplier();
        }));
        // right center button
        DRIVER.start().onTrue(this.drivetrain.runOnce(() -> {
            this.drivetrain.increaseSpeedMultiplier();
        }));
        // TODO -- furthermore, we can implement "zoned" speeds using the pose estimator

        /* Configure Climb */
        DRIVER.a().onTrue(this.m_climber.releaseClimbWinch());
        DRIVER.rightBumper().whileTrue(this.m_climber.winchUp())
                .onFalse(this.m_climber.stopWinch());
        DRIVER.rightTrigger().whileTrue(this.m_climber.winchDown())
                .onFalse(this.m_climber.stopWinch());
    }

    private void configureOperatorBindings() {
        // TODO see if the elevator works

        // TODO POV should be the D-Pad, but check that this is correct

        /* Configure Elevator */
        OPERATOR.rightStick()
                .onChange(this.m_elevator.applySpeed(() -> -1 * OPERATOR.getRightY() / 2));
        // on the controller: up == -1, down == 1

        /* Configure Pivot */
        OPERATOR.leftStick().onChange(this.m_pivot.applySpeeds(() -> -1 * OPERATOR.getLeftY() / 2));
        // on the controller: up == -1, down == 1

        /* Configure joint Elevator/Pivot positioning */
        OPERATOR.a().onTrue(this.m_elevator.L1().andThen(this.m_pivot.L1()));
        OPERATOR.b().onTrue(this.m_elevator.L2().andThen(this.m_pivot.L2()));
        OPERATOR.x().onTrue(this.m_elevator.L3().andThen(this.m_pivot.L3()));
        OPERATOR.y().onTrue(this.m_elevator.L4().andThen(this.m_pivot.L4()));

        /* Configure CoralHandler */
        OPERATOR.leftBumper().onTrue(this.m_coralHandler.intakeFromStation())
                .onFalse(this.m_coralHandler.stop());
        OPERATOR.leftTrigger().onTrue(this.m_coralHandler.spitItOut())
                .onFalse(this.m_coralHandler.stop());

        /* Configure AlgaeHandler */
        OPERATOR.rightBumper().onTrue(this.m_algaeHandler.rotateCW())
                .onFalse(this.m_algaeHandler.stop());
        OPERATOR.rightTrigger().onTrue(this.m_algaeHandler.rotateCCW())
                .onFalse(this.m_algaeHandler.stop());
    }

    public Command getAutonomousCommand() {
        // reset the poseEstimator's initialPoseMeters to the AutoBuilder's pose // TODO verify
        this.m_poseEstimator.resetPose(AutoBuilder.getCurrentPose());
        // return the autoChooser's selected Auto
        return this.autoChooser.getSelected();
    }

    public Pose2d getEstimatedPosition() {
        return this.m_poseEstimator.getEstimatedPosition();
    }
}
