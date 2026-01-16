// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.AprilTagTracker;
import frc.robot.subsystems.vision.LimeLight.PoseEstimate;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#
 * defining-commands
 */
public class AlignTag extends Command {
        private static double targetRotation = 0; // What degree we want our swerve to turn to.
        
        // Lets us adjust rotational speed depending on our rotation's error.
        private static PIDController rotationPID = new PIDController(1.0, 0.0, 0.1);

        // Might work.
        // Every interval the B button is pressed, we'll keep rotating until we're facing the target.
        public static Command alignToTag(CommandSwerveDrivetrain m_drivetrain, AprilTagTracker m_april_tag) {
                PoseEstimate current_tag = m_april_tag.get_latest_pose();
                Rotation2d current_rot = current_tag.pose().getRotation();
                double rotation_error = targetRotation - current_rot.getRadians();
                double rotationalSpeed = rotationPID.calculate(rotation_error);

                return m_drivetrain.applyRequest(
                                () -> new SwerveRequest.FieldCentric().withRotationalRate(rotationalSpeed));
        }
}
