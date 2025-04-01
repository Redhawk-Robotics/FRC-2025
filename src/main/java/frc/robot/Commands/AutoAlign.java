// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.nio.channels.Pipe.SourceChannel;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.CoralAligner;

// notes
// let's turn this into a Factory?

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign extends Command {
  /** Creates a new AutoAlign.  We use time of flight sensors here */
  
  private CommandSwerveDrivetrain m_drivetrain;
  private CoralAligner m_coralAligner;

  public AutoAlign(CommandSwerveDrivetrain m_drivetrain, CoralAligner m_vision) {
    this.m_drivetrain = m_drivetrain;
    this.m_coralAligner = m_coralAligner;

    addRequirements(m_drivetrain, m_coralAligner);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println( "Interrupted, Command Finishing");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // if command selected to go left and leftdesired state is true, end
    //same with other
    // https://github.com/Redhawk-Robotics/FRC-2024/blob/main/src/main/java/frc/robot/commands/automation/IntakeToShooter.java
    // Using this becuase it worked prettty well last year 
    return false;
  }
}
