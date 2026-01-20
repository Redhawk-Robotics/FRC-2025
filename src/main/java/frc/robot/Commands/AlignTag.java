// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.AprilTagTracker;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#
 * defining-commands
 */
public class AlignTag {

        static double PID_TOLERANCE = 2.0;

        // Might work.
        // Every interval the B button is pressed, we'll keep rotating until we're facing the target.
        public static Command alignToTag(CommandSwerveDrivetrain m_drivetrain, AprilTagTracker m_april_tag) {
                PIDController PID = new PIDController(1.0, 0.0, 0.1);
                PID.enableContinuousInput(-Math.PI, Math.PI);
                PID.setTolerance(Math.toRadians(PID_TOLERANCE));

                return m_drivetrain.applyRequest(() -> {
                        // TODO:
                        // * check if PID is created on every B press

                        double desired_angle = m_april_tag.get_latest_pose().pose().getRotation().getRadians();
                        double rotationalSpeed = PID.calculate(desired_angle, 0);
                        
                        return new SwerveRequest.FieldCentric().withRotationalRate(rotationalSpeed);
                }).until(PID::atSetpoint).finallyDo(() -> {
                        PID.close();
                });
        }
}
