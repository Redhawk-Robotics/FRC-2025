// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.nio.channels.Pipe.SourceChannel;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.CANRanges;

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
    // .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05)
    // .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    /*
     * First, the swerve needs to rotate to become paralell to the reef, but given that we're going
     * to be flush against the reef, I guess we can skip this step for now.
     * 
     * Goal - depending on a button the driver presses, the swerve base begins going left / right
     * until the boolean isAlignedLeft() / isAlignedRight() from the CoralAligner subsystem is
     * satisfied.
     */

    //todo test alignment to the right first, then implement that logic for the right reef
    public static Command alignToLeftReef(CommandSwerveDrivetrain m_drivetrain, CANRanges m_CANRanges) {

        //! something wrong with the swerve request
        return stopDrive(m_drivetrain).raceWith(Commands.waitSeconds(0.1))
                .andThen(applySwerveRequest(m_drivetrain, -DRIVE_RATE).raceWith(Commands.waitUntil(m_CANRanges::isAlignedLeft)))
                .andThen(stopDrive(m_drivetrain).raceWith(Commands.waitSeconds(0.1)))
                .withName("Aligning Left...");

        // return new FunctionalCommand(
        //         //first stops the current drive
        //         () -> stopDrive(m_drivetrain),
        //         //starts driving left / right
        //         () -> applySwerveRequest(m_drivetrain, -DRIVE_RATE),
        //         //supposed to be a boolean consumer that handles the interruption
        //         interrupt -> stopDrive(m_drivetrain),
        //         //boolean that determines if the command ended
        //         () -> m_CANRanges.isAlignedRight(),
        //         //command requirement that is filled with the parameter
        //         m_drivetrain).withName("Aligning Left...");
        // return Commands.run(() -> {
        //     applySwerveRequest(m_drivetrain, DRIVE_RATE);
        // }, m_drivetrain, m_CANRanges);
    }

    public static Command alignToRightReef(CommandSwerveDrivetrain m_drivetrain,
            CANRanges m_CANRanges) {
                return stopDrive(m_drivetrain).raceWith(Commands.waitSeconds(0.1))
                .andThen(applySwerveRequest(m_drivetrain, DRIVE_RATE).raceWith(Commands.waitUntil(m_CANRanges::isAlignedRight)))
                .andThen(stopDrive(m_drivetrain).raceWith(Commands.waitSeconds(0.1)))
                .withName("Aligning Right...");
    }

    //! not sure about this
    public static Command stopDrive(CommandSwerveDrivetrain m_drivetrain) {
        return m_drivetrain.applyRequest(() -> AutoAlign.brake);
    }

    //todo test these values
    public static Command applySwerveRequest(CommandSwerveDrivetrain m_drivetrain, double speeds) {
        return m_drivetrain
                .applyRequest(() -> drive.withVelocityY(speeds) //todo confirm direction
                        .withVelocityX(0).withRotationalRate(0));
    }

    private static SwerveRequest.RobotCentric getRobotCentricRequest(double speeds) {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        return drive//
                .withVelocityX(speeds).withVelocityY(0).withRotationalRate(0);
    }

}
