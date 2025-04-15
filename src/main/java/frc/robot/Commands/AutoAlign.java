// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CANRanges;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

// notes
// let's turn this into a Factory?

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#
 * defining-commands
 */
public class AutoAlign {
    /** Creates a new AutoAlign. We use time of flight sensors here */

    private static final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();//
    private static final SwerveRequest.SwerveDriveBrake brake =
            new SwerveRequest.SwerveDriveBrake();
    private static double DRIVE_RATE = 0.3;

    /*
     * First, the swerve needs to rotate to become paralell to the reef, but given that we're going
     * to be flush against the reef, I guess we can skip this step for now.
     * 
     * Goal - depending on a button the driver presses, the swerve base begins going left / right
     * until the boolean isAlignedLeft() / isAlignedRight() from the CoralAligner subsystem is
     * satisfied.
     */

    public static Command alignToLeftReef(CommandSwerveDrivetrain m_drivetrain,
            CANRanges m_CANRanges) {

        return stopDrive(m_drivetrain).raceWith(Commands.waitSeconds(0.1))
                .andThen(applySwerveRequest(m_drivetrain, -DRIVE_RATE)
                        .raceWith(Commands.waitUntil(m_CANRanges::isAlignedLeft)))
                .andThen(stopDrive(m_drivetrain).raceWith(Commands.waitSeconds(0.1)))
                .withName("Aligning Left...");
    }

    public static Command alignToRightReef(CommandSwerveDrivetrain m_drivetrain,
            CANRanges m_CANRanges) {

        return stopDrive(m_drivetrain).raceWith(Commands.waitSeconds(0.1))
                .andThen(applySwerveRequest(m_drivetrain, DRIVE_RATE)
                        .raceWith(Commands.waitUntil(m_CANRanges::isAlignedRight)))
                .andThen(stopDrive(m_drivetrain).raceWith(Commands.waitSeconds(0.1)))
                .withName("Aligning Right...");
    }

    //! not sure about this
    public static Command stopDrive(CommandSwerveDrivetrain m_drivetrain) {
        return m_drivetrain.applyRequest(() -> AutoAlign.brake);
    }

    //todo test these values
    public static Command applySwerveRequest(CommandSwerveDrivetrain m_drivetrain, double speeds) {
        return m_drivetrain.applyRequest(
                () -> drive.withVelocityY(speeds).withVelocityX(0).withRotationalRate(0));
    }
}
