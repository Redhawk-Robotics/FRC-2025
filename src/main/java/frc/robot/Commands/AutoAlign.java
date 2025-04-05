// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.nio.channels.Pipe.SourceChannel;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.CANRanges;

// notes
// let's turn this into a Factory? 

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign {
  /** Creates a new AutoAlign.  We use time of flight sensors here */


  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();//
            // .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05)
            // .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    /*
     * First, the swerve needs to rotate to become paralell to the reef, but given that we're going to be 
     * flush against the reef, I guess we can skip this step for now.
     * 
     * Goal - depending on a button the driver presses, the swerve base begins going left / right until the
     * boolean isAlignedLeft() / isAlignedRight() from the CoralAligner subsystem is satisfied.
     */
  public Command alignToLeftReef(CommandSwerveDrivetrain m_drivetrain, CANRanges m_CoralAligner){
    return driveLeft(m_drivetrain);
  }

  public Command alignToRightReef(CommandSwerveDrivetrain m_drivetrain, CANRanges m_CoralAligner) {
    return Commands.none();
  }

  //! not sure about this
  public Command stopDrive( CommandSwerveDrivetrain m_drivetrain) {
    return m_drivetrain.applyRequest(() -> drive.withVelocityX(-1)).withName("@@@@@@@@@@@@@@Stopping Drive Train"); 
  }

  //todo test these values
  public Command driveLeft( CommandSwerveDrivetrain m_drivetrain) {
    return m_drivetrain.applyRequest(() -> drive.withVelocityX(0.2)).withName("@@@@@@@@@@@@@@Aligning Left");
  }

  public Command driveRight( CommandSwerveDrivetrain m_drivetrain) {
    return m_drivetrain.applyRequest(() -> drive.withVelocityX(-0.2)).withName("@@@@@@@@@@@@@@Aligning Right...");
  }

  // !not sure how i should be making this command ..?
  public Command makeSwerveParalell(CommandSwerveDrivetrain m_drivetrain){
    return Commands.none(); //m_drivetrain.getRotation3d();
  } 


  /*
    return this.drive//
                .withVelocityX( // Drive forward with positive Y (forward)
                        MathUtil.applyDeadband(this.DRIVER.getLeftY(), 0.1)//
                                * this.MaxSpeed * this.drivetrain.speedMultiplier())
                .withVelocityY( // Drive left with positive X (left)
                        MathUtil.applyDeadband(this.DRIVER.getLeftX(), 0.1)//
                                * this.MaxSpeed * this.drivetrain.speedMultiplier())
                .withRotationalRate( // Drive counterclockwise with negative X (left)
                        MathUtil.applyDeadband(-this.DRIVER.getRightX(), 0.1)//
                                * this.MaxAngularRate * this.drivetrain.speedMultiplier());
   */
  
}
