// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CANRanges;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.AprilTagTracker;
import frc.robot.subsystems.vision.LimeLight.PoseEstimate;

// notes
// let's turn this into a Factory?

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#
 * defining-commands
 */
public class AlignTag extends Command {
        /** Creates a new AutoAlign. We use time of flight sensors here */

        private static final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();//
        private static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private static double DRIVE_RATE = 0.3;

        private static double targetRotation = 0; // The target rotation in radians (e.g., from a vision system)
        private static PIDController rotationPID = new PIDController(1.0, 0.0, 0.1); // Adjust PID constants as needed

        /*
         * First, the swerve needs to rotate to become paralell to the reef, but given
         * that we're going
         * to be flush against the reef, I guess we can skip this step for now.
         * 
         * Goal - depending on a button the driver presses, the swerve base begins going
         * left / right
         * until the boolean isAlignedLeft() / isAlignedRight() from the CoralAligner
         * subsystem is
         * satisfied.
         */

        // alligns
        public static Command alignToTag(CommandSwerveDrivetrain m_drivetrain, AprilTagTracker m_april_tag) {
                PoseEstimate current_tag = m_april_tag.get_latest_pose();
                Rotation2d current_rot = current_tag.pose().getRotation();
                double rotation_error = targetRotation - current_rot.getRadians();
                double rotationalSpeed = rotationPID.calculate(rotation_error);

                // TODO: Make it rotate towards the latest pose
                return m_drivetrain.applyRequest(
                                () -> new SwerveRequest.FieldCentric().withRotationalRate(rotationalSpeed));
        }
}
