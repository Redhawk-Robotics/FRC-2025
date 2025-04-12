// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.sendables.Field;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#
 * defining-commands
 */
public class DriveToPose extends Command {

    private CommandSwerveDrivetrain drivetrain;
    private Pose2d target;

    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/holonomic.html
    private final HolonomicDriveController controller = new HolonomicDriveController(//
            new PIDController(20, 1, 2), // same as PathPlanner
            new PIDController(20, 1, 2), //
            new ProfiledPIDController(20, 1, 2, //
                    new TrapezoidProfile.Constraints(360, 360)));

    private final ApplyRobotSpeeds robotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/filters/debouncer.html#modes
    // rising == false->true only
    private final Trigger isAligned =
            new Trigger(this::aligned).debounce(0.15, DebounceType.kRising);

    private final boolean aligned() {
        return Math.abs(this.drivetrain.getPose().getX() - this.target.getX()) < 0.1
                && Math.abs(this.drivetrain.getPose().getY() - this.target.getY()) < 0.1
                && (this.drivetrain.getPose().getRotation().getDegrees()
                        - this.target.getRotation().getDegrees()) % 360 < 0.5;
    }


    /** Creates a new DriveToPose. */
    public DriveToPose(CommandSwerveDrivetrain drivetrain, Pose2d target) {
        this.drivetrain = drivetrain;
        this.target = target;


        this.addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Field.field.getObject("DriveToPose.start").setPose(this.drivetrain.getPose());
        Field.field.getObject("DriveToPose.goal").setPose(this.target);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double desiredLinearVelocity =
                DriveToPose.zonedVelocity(this.drivetrain.getPose().getTranslation()); // todo -- maybe this should be dependent on the field position
        desiredLinearVelocity = 1;

        // cull it if we're close

        var speeds = this.controller.calculate(this.drivetrain.getPose(), this.target,
                desiredLinearVelocity, this.target.getRotation());

        this.drivetrain.setControl(this.robotSpeeds.withSpeeds(speeds));
    }

    // "field_size":{"x":17.548,"y":8.052}
    // 17.5m x 8m
    // half-meter bins
    // 35 x 16 bins
    private static double[][] zones = new double[][] {//
            {0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                    0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.}, //
            {0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                    0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.}, //
            {0., 0., 0., 0., 0., 0., 0., 4., 4., 4., 4., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                    0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.}, //
            {0., 0., 0., 0., 0., 0., 0., 4., 4., 4., 4., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                    0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.}, //
            {0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                    0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.}, //
            {0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                    0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.}, //
            {0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                    0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.}, //
            {0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                    0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.}, //
            {0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                    0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.}, //
            {0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                    0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.}, //
            {0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                    0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.}, //
            {0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                    0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.}, //
            {0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                    0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.}, //
            {0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                    0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.}, //
            {0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                    0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.}, //
            {0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                    0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.}};

    private static double zonedVelocity(Translation2d state) {
        // get current Translation2d
        // send to array indices in  zones
        // the X component 
        // (0,0) == lower left corner
        // (17.5,8) == upper right corner
        int row = (int) Math.floor((8.053 - state.getY()) * 2);
        if (row < 0) {
            row = 0;
        } else if (row >= DriveToPose.zones.length) {
            row = DriveToPose.zones.length - 1;
        }
        int col = (int) Math.floor(state.getX() * 2);
        if (col < 0) {
            col = 0;
        } else if (col >= DriveToPose.zones[0].length) {
            col = DriveToPose.zones[0].length - 1;
        }
        return DriveToPose.zones[row][col];
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Field.field.getObject("DriveToPose.start").close();
        Field.field.getObject("DriveToPose.goal").close();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.isAligned.getAsBoolean();
    }
}
