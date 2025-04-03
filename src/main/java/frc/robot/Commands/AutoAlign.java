// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.nio.channels.Pipe.SourceChannel;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.CoralAligner;

// notes
// let's turn this into a Factory? 

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign {
  /** Creates a new AutoAlign.  We use time of flight sensors here */

  public Command alignToLeftReef(CommandSwerveDrivetrain m_drivetrain, CoralAligner m_CoralAligner){
    return Commands.none();
  }

  public Command alignToRightReef(CommandSwerveDrivetrain m_Drivetrain, CoralAligner m_CoralAligner) {
    return Commands.none();
  }
}
