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
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Commands.CoralPositionFactory;
import frc.robot.Constants.Ports;
import frc.robot.Constants.Settings;
import frc.robot.Constants.Settings.CoralPosition;
import frc.robot.subsystems.Pivot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeFloorIntake;
import frc.robot.subsystems.AlgaeHandler;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralHandler;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.AlgaeFloorIntake.positions;
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
    private final AlgaeFloorIntake m_algaeFloorIntake = new AlgaeFloorIntake();

    private final Orchestra m_orchestra = new Orchestra();

    private class SendablePID implements Sendable {
        private boolean isTuningMode = false;
        //
        private float elevator_kP1 = (float) Settings.Elevator.kP_UP;
        private float elevator_kI1 = (float) Settings.Elevator.kI_UP;
        private float elevator_kD1 = (float) Settings.Elevator.kD_UP;
        //
        private float elevator_kP2 = (float) Settings.Elevator.kP_DOWN;
        private float elevator_kI2 = (float) Settings.Elevator.kI_DOWN;
        private float elevator_kD2 = (float) Settings.Elevator.kD_DOWN;
        //
        private float pivot_kP = (float) Settings.Pivot.kP;
        private float pivot_kI = (float) Settings.Pivot.kI;
        private float pivot_kD = (float) Settings.Pivot.kD;
        //
        private float elevator_sp = 20;
        private float pivot_sp = 100;
 
        private static float clamp(float val, float low, float high) {
            if (val < low)
                return low;
            if (val > high)
                return high;
            return val;
        }

        public boolean isTuningMode() {
            return this.isTuningMode;
        }

        public float getElevator_kP1() {
            return this.elevator_kP1;
        }

        public float getElevator_kI1() {
            return this.elevator_kI1;
        }

        public float getElevator_kD1() {
            return this.elevator_kD1;
        }

        public float getElevator_kP2() {
            return elevator_kP2;
        }

        public float getElevator_kI2() {
            return elevator_kI2;
        }

        public float getElevator_kD2() {
            return elevator_kD2;
        }

        public float getElevator_setpoint() {
            return this.elevator_sp;
        }

        public float getPivot_kP() {
            return this.pivot_kP;
        }

        public float getPivot_kI() {
            return this.pivot_kI;
        }

        public float getPivot_kD() {
            return this.pivot_kD;
        }

        public float getPivot_setpoint() {
            return this.pivot_sp;
        }

        @Override
        public void initSendable(SendableBuilder builder) {
            // setter = SmartDashboard is giving the Robot a value
            // getter = SmartDashboard is requesting the current value

            builder.addBooleanProperty("Tuning Mode", () -> this.isTuningMode,
                    val -> this.isTuningMode = val);
            //
            builder.addFloatProperty("Elevator/kP-up", () -> this.elevator_kP1,
                    val -> this.elevator_kP1 = SendablePID.clamp(val, 0, 1000));
            builder.addFloatProperty("Elevator/kI-up", () -> this.elevator_kI1,
                    val -> this.elevator_kI1 = SendablePID.clamp(val, 0, 1000));
            builder.addFloatProperty("Elevator/kD-up", () -> this.elevator_kD1,
                    val -> this.elevator_kD1 = SendablePID.clamp(val, 0, 1000));
            //
            builder.addFloatProperty("Elevator/kP-down", () -> this.elevator_kP2,
                    val -> this.elevator_kP1 = SendablePID.clamp(val, 0, 1000));
            builder.addFloatProperty("Elevator/kI-down", () -> this.elevator_kI2,
                    val -> this.elevator_kI1 = SendablePID.clamp(val, 0, 1000));
            builder.addFloatProperty("Elevator/kD-down", () -> this.elevator_kD2,
                    val -> this.elevator_kD1 = SendablePID.clamp(val, 0, 1000));
            //
            builder.addFloatProperty("Pivot/kP", () -> this.pivot_kP,
                    val -> this.pivot_kP = SendablePID.clamp(val, 0, 1000));
            builder.addFloatProperty("Pivot/kI", () -> this.pivot_kI,
                    val -> this.pivot_kI = SendablePID.clamp(val, 0, 1000));
            builder.addFloatProperty("Pivot/kD", () -> this.pivot_kD,
                    val -> this.pivot_kD = SendablePID.clamp(val, 0, 1000));
            //
            builder.addFloatProperty("Elevator/setpoint", () -> this.elevator_sp,
                    val -> this.elevator_sp = val);
            builder.addFloatProperty("Pivot/setpoint", () -> this.pivot_sp,
                    val -> this.pivot_sp = val);
        }
    }

    private final SendablePID m_PID = new SendablePID();


    public RobotContainer() {
        configureBindings();

        SmartDashboard.putData("Auto Mode", autoChooser);

        this.drivetrain.setPoseUpdater(//
                t -> this.m_poseEstimator.update(t.getFirst(), t.getSecond()));
        // t -> this.m_poseEstimator.updateWithTime(Timer.getFPGATimestamp(), t.getFirst(),
        // t.getSecond())); // TODO should we use this one?
        drivetrain.registerTelemetry(logger::telemeterize);

        // TODO remove once tuned
        SmartDashboard.putData("PID", this.m_PID);

        if (false) { // todo ignore
            configureMusic();
        }
    }

    private void configureMusic() {
        for (SwerveModule<TalonFX, TalonFX, CANcoder> module : this.drivetrain.getModules()) {
            m_orchestra.addInstrument(module.getDriveMotor());
            m_orchestra.addInstrument(module.getSteerMotor());
        }
        var status = m_orchestra.loadMusic(Filesystem.getDeployDirectory() + "/chrp/wii-shop.chrp");
        if (!status.isOK()) {
            // log error
            DriverStation.reportError(status.toString(), Thread.currentThread().getStackTrace());
        } else {
            System.out.println(m_orchestra.play());
        }
        System.out.println(m_orchestra.isPlaying());
    }

    private void configureBindings() {
        this.configureDriverBindings();
        this.configureOperatorBindings();
        this.configureNamedCommands();
    }

    private SwerveRequest.FieldCentric getFieldCentricDrive() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        return drive//
                .withVelocityX( // Drive forward with positive Y (forward)
                        Math.pow(MathUtil.applyDeadband(DRIVER.getLeftY(), 0.1), 5)//
                                * MaxSpeed * drivetrain.speedMultiplier())
                .withVelocityY( // Drive left with positive X (left)
                        Math.pow(MathUtil.applyDeadband(DRIVER.getLeftX(), 0.1), 5)//
                                * MaxSpeed * drivetrain.speedMultiplier())
                .withRotationalRate( // Drive counterclockwise with negative X (left)
                        Math.pow(MathUtil.applyDeadband(-DRIVER.getRightX(), 0.1), 5)//
                                * MaxAngularRate * drivetrain.speedMultiplier());
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

        // && DRIVER BACK AND Y 
        // * Starts SYSID dynamic directions
        DRIVER.back().and(DRIVER.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));

        // && DRIVER BACK AND X
        // * Starts SYSID dynamic directions
        DRIVER.back().and(DRIVER.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));

        // && START BACK AND Y 
        // * Starts SYSID dynamic directions
        DRIVER.start().and(DRIVER.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));

        // && START AND X 
        // * TOGGLES REVERSE
        DRIVER.start().and(DRIVER.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // && LEFT BUMPER
        // * RESET FIELD CENTRIC DRIVE
        // reset the field-centric heading on left bumper press
        DRIVER.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //& 
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

        /* Configure joing Elevator/Pivot positioning */
        DRIVER.x().onTrue(CoralPositionFactory.Feed(this.m_elevator, this.m_pivot));

        // & DRIVER UP AND DOWN CONTRLLS THE ALGAE INTAKE
        // ! ALGAE INTAKE NEEDS TO BE FIXED
        //TODO FLOOR INTAKE START END -- DPAD UP IS MOVE OUT AND ROLLER -- DPAD DOWN IS OUTAKE WITH NO ARM MOVEMENT
        // DRIVER.povUp().whileTrue(
        //     this.m_algaeFloorIntake.startEnd(
        //         () -> this.m_algaeFloorIntake.intakeAlgae(), 
        //         () -> this.m_algaeFloorIntake.returnHome()
        //     ));

        // DRIVER.povDown().whileTrue(
        //     this.m_algaeFloorIntake.startEnd(
        //     ()-> this.m_algaeFloorIntake.runRollerCCW(), 
        //     () -> this.m_algaeFloorIntake.stopRoller())
        // );
    }

    public void zero() {
        this.m_elevator.stopElevator();
        this.m_pivot.stopPivot();
    }

    private void configureOperatorBindings() {

        
        /* Configure Elevator */
        Command elevatorDefault = Commands.either(//
                Commands.none(), //
                this.m_elevator.runOnce(//
                        () -> this.m_elevator.setSpeed(
                                MathUtil.applyDeadband((-1. * OPERATOR.getLeftY()), 0.1) / 2.25)), //
                this.m_PID::isTuningMode);
        elevatorDefault.addRequirements(m_elevator);
        this.m_elevator.setDefaultCommand(elevatorDefault);
        // on the controller: up == -1, down == 1

        /* Configure Pivot */
        Command pivotDefault = Commands.either(//
                Commands.none(), //
                this.m_pivot.runOnce(//
                        () -> this.m_pivot.setSpeed(
                                MathUtil.applyDeadband((-1. * OPERATOR.getRightY()), 0.1) / 2.25)), //
                this.m_PID::isTuningMode);
        pivotDefault.addRequirements(m_pivot);
        this.m_pivot.setDefaultCommand(pivotDefault);
        // // on the controller: up == -1, down == 1

        /* Configure joint Elevator/Pivot positioning */
        // tuning mode:
        // a == re-flash elevator motors (with new PID values)
        // b == elevator go to setpoint
        // x == re-flash pivot motor (with new PID values)
        // y == pivot go to setpoint

        OPERATOR.a().whileTrue( CoralPositionFactory.L1(m_elevator, m_pivot)).
        toggleOnFalse( CoralPositionFactory.Feed(m_elevator, m_pivot));

        OPERATOR.b().whileTrue( CoralPositionFactory.L2(m_elevator, m_pivot)).
        toggleOnFalse( CoralPositionFactory.Feed(m_elevator, m_pivot));

        OPERATOR.x().whileTrue( CoralPositionFactory.L3(m_elevator, m_pivot)).
        toggleOnFalse( CoralPositionFactory.Feed(m_elevator, m_pivot));

        OPERATOR.y().whileTrue( CoralPositionFactory.L4(m_elevator, m_pivot)).
        toggleOnFalse( CoralPositionFactory.Feed(m_elevator, m_pivot));

        


        //TODO DEPRECATED, PID CONTROL
        //&& A BUTTON 
        // *CONFIGURES THE MOTORS WITH PID VALUES FOR ELEVATOR ONLY IF TUNING MODE IS ON
        // * IF TUNER MODE IS OFF IT RUNS TO L1
        // OPERATOR.a().whileTrue(Commands.either(//
        //         this.m_elevator.runOnce(//
        //                 () -> this.m_elevator.configureMotors(//
        //                         // tune Elevator PID UP
        //                         this.m_PID.getElevator_kP1(), this.m_PID.getElevator_kI1(),
        //                         this.m_PID.getElevator_kD1(), //
        //                         // tune Elevator PID DOWN
        //                         this.m_PID.getElevator_kP2(), this.m_PID.getElevator_kI2(),
        //                         this.m_PID.getElevator_kD2())), //
        //         CoralPositionFactory.L1(m_elevator, m_pivot).handleInterrupt(
        //              ()-> CoralPositionFactory.Feed(m_elevator, m_pivot)), //
        //         this.m_PID::isTuningMode));

        OPERATOR.a().whileTrue( CoralPositionFactory.L1(m_elevator, m_pivot)).toggleOnFalse( CoralPositionFactory.Feed(m_elevator, m_pivot));

        CoralPositionFactory.L4(m_elevator, m_pivot).handleInterrupt( ()-> CoralPositionFactory.Feed(m_elevator, m_pivot));
        //&& OPERATOR B BUTTON 
        // * IF TUNER MODE IS ON, IT SETS REFERENCE TO THE SETPOINT
        // * IF TUNRE MODE IS OFF JUST RUNS TO L2 

        // OPERATOR.b().whileTrue(Commands.either(//
        //         this.m_elevator.startEnd(
        //                 () -> this.m_elevator.setReference(this.m_PID.getElevator_setpoint()),
        //                 () -> this.m_elevator.stopElevator()),
        //         CoralPositionFactory.L2(this.m_elevator, this.m_pivot), //
        //         this.m_PID::isTuningMode));
        
        //&& OPERATOR X BUTTON
        // * IF THIS IS IN TUNER MODE, THIS CHANGES THE PIVOT PID VALUES
        // * OTHERWISE, IT RUNS TO L3 POSITION FOR BOTH PIVOT AND ELEVATOR
        // OPERATOR.x().whileTrue(Commands.either(//
        //         this.m_pivot.runOnce(//
        //                 () -> this.m_pivot.configureMotors(// tune Pivot (done?)
        //                         this.m_PID.getPivot_kP(), this.m_PID.getPivot_kI(),
        //                         this.m_PID.getPivot_kD())), //
        //         CoralPositionFactory.L3(this.m_elevator, this.m_pivot), //
        //         this.m_PID::isTuningMode));
        
        // && OPERATOR Y 
        // * BUTTON TO THE REFERENCE SET IN THE TUNER DASHBOARD
        // * OTHERWISE, RUNS TO L4
        // OPERATOR.y().whileTrue(Commands.either(//
        //         this.m_pivot.startEnd(
        //                 () -> this.m_pivot.setReference(this.m_PID.getPivot_setpoint()),
        //                 () -> this.m_pivot.stopPivot()),
        //         CoralPositionFactory.L4(this.m_elevator, this.m_pivot), //
        //         this.m_PID::isTuningMode));

        // OPERATOR.a().onTrue(CoralPositionFactory.L1(this.m_elevator, this.m_pivot));
        // OPERATOR.b().onTrue(CoralPositionFactory.L2(this.m_elevator, this.m_pivot));
        // OPERATOR.x().onTrue(CoralPositionFactory.L3(this.m_elevator, this.m_pivot));
        // OPERATOR.y().onTrue(CoralPositionFactory.L4(this.m_elevator, this.m_pivot));

        /* Configure CoralHandler */

        //  & OPERATOR LEFT BUMPER
        // * Intakes coral
        OPERATOR.leftBumper().onTrue(this.m_coralHandler.intakeFromStation())
                .onFalse(this.m_coralHandler.stop());
        
        //& OPERATOR LEFT TRIGGER
        // * Spits out coral 
        OPERATOR.leftTrigger().onTrue(this.m_coralHandler.spitItOut())
                .onFalse(this.m_coralHandler.stop());

        //& RIGHT BUMPERS
        //* ALGAE HANDLER, N/A */
        /* Configure AlgaeHandler */
        OPERATOR.rightBumper().onTrue(this.m_algaeHandler.rotateCW())
                .onFalse(this.m_algaeHandler.stop());
        OPERATOR.rightTrigger().onTrue(this.m_algaeHandler.rotateCCW())
                .onFalse(this.m_algaeHandler.stop());

        /*  Algae Intake */

        // on d-pad down, zero the current elevator position
        // OPERATOR.povDown().onTrue(//
        //         this.m_elevator.runOnce(() -> this.m_elevator.resetElevatorPosition()));
    
        // on d-pad up, tell the elevator and pivot to use _speed_ control
        // with the joysticks, instead of PID position control
        OPERATOR.povUp().onTrue(//
                Commands.parallel(//
                        this.m_elevator.runOnce(() -> this.m_elevator.useSpeed()), //
                        this.m_pivot.runOnce(() -> this.m_pivot.useSpeed())));
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

    public void configureNamedCommands() {

        // && POSITIONS 
        NamedCommands.registerCommand("L1 Position", CoralPositionFactory.L1(m_elevator, m_pivot));
        NamedCommands.registerCommand("L2 Position", CoralPositionFactory.L2(m_elevator, m_pivot));
        NamedCommands.registerCommand("L3 Position", CoralPositionFactory.L3(m_elevator, m_pivot));
        NamedCommands.registerCommand("L4 Position", CoralPositionFactory.L4(m_elevator, m_pivot));
        NamedCommands.registerCommand("Feeder Position", CoralPositionFactory.Feed(m_elevator, m_pivot));

        // TODO THESE MUST BE TESTED
        NamedCommands.registerCommand("Intake Coral", m_coralHandler.commandIntakeCoral());
        NamedCommands.registerCommand("Intake Coral", m_coralHandler.commandOutTakeCoral());

    }
}
