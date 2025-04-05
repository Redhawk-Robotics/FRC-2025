// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.subsystems.AlgaeFloorIntake;
import frc.robot.subsystems.AlgaeHandler;
import frc.robot.subsystems.CoralHandler;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.CANRanges;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

/** Add your docs here. */
public class roboLogger {

    DataLog log;
    
    CommandSwerveDrivetrain m_drivetrain;
    AlgaeFloorIntake m_algaeFloorIntake;
    AlgaeHandler m_algaeHandler;
    CANRanges m_canRanges;
    CoralHandler m_coralHandler;
    Elevator m_elevator;
    Pivot m_pivot;
    Vision m_vision;


    //elevator
    DoubleLogEntry elevatorPositionLog;


    //pivot
    DoubleLogEntry pivotPositionLog;

    // * i am not done writing!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // * i want to send logs from other classes as well

    //Put all subsystems as parameters maybe? Even if this doesn't necessarily work for the position factory w/o a little refactorinig
    // This gives us opportunities for match logging.
    public roboLogger( CommandSwerveDrivetrain m_drivetrain, AlgaeFloorIntake m_algaeFloorIntake, 
                    AlgaeHandler m_algaeHandler, CANRanges m_canRanges, CoralHandler m_coralHandler,
                    Elevator m_elevator, Pivot m_pivot, Vision m_vision) {

        DataLogManager.start();
        log = DataLogManager.getLog();

        this.m_drivetrain = m_drivetrain;
        this.m_algaeFloorIntake = m_algaeFloorIntake;
        this.m_algaeHandler = m_algaeHandler;
        this.m_canRanges = m_canRanges;
        this.m_coralHandler = m_coralHandler;
        this.m_elevator = m_elevator;
        this.m_pivot = m_pivot;
        this.m_vision = m_vision;

        //Init elevator logging
        elevatorPositionLog = new DoubleLogEntry(log, "/Elevator/Position");

        //Init Pivot Position Logging
        pivotPositionLog = new DoubleLogEntry(log, "/Pivot/Position");
    }

    //hmm what do i need for this
    // public void sendLog() {}
    // public void sendLog() {}
    // public void sendLog() {}

    public void logElevatorDataPeriodic() {
        elevatorPositionLog.append( m_elevator.getPosition());
        //elevatorPositionLog.update(0);

        // elevatorPosition.update(m, 0);
        // myStringLog.append(null);
    }
    public void logPivotDataPeriodic() {
        //more to be done tomorrow 
        pivotPositionLog.append( m_pivot.getPosition());
    }
}
