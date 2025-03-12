// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import org.opencv.core.Point;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Settings;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralPositionFactory2 extends Command {
  /** Creates a new CoralPositionFactory2. */


  Pivot m_Pivot;
  Elevator m_Elevator;

  position currentPosition;


  public enum position {

    F0(Settings.CoralPosition.ELEVATOR_FEED_POSITION,
    Settings.CoralPosition.PIVOT_FEED_POSITION), //
    L1(Settings.CoralPosition.ELEVATOR_L1_POSITION, Settings.CoralPosition.PIVOT_L1_POSITION), //
    L2(Settings.CoralPosition.ELEVATOR_L2_POSITION, Settings.CoralPosition.PIVOT_L2_POSITION), //
    L3(Settings.CoralPosition.ELEVATOR_L3_POSITION, Settings.CoralPosition.PIVOT_L3_POSITION), //
    L4(Settings.CoralPosition.ELEVATOR_L4_POSITION, Settings.CoralPosition.PIVOT_L4_POSITION);

    private final double elevPos;
    private final double pivotPos;

    private position(double e, double p) {
        this.elevPos = e;
        this.pivotPos = p;
    }
    public double elevatorPosition() {
        return this.elevPos;
    }
    public double pivotPosition() {
        return this.pivotPos;
    }
  }

  public CoralPositionFactory2(Pivot m_Pivot, Elevator m_Elevator, position m_position) {
    this.m_Pivot = m_Pivot;
    this.m_Elevator = m_Elevator;

    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_Pivot.setReference(position.F0.pivotPos);
    // m_Elevator.setReference(position.F0.elevPos);
    m_Pivot.runOnce( () -> {m_Pivot.setReference(position.F0.pivotPos);} );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
