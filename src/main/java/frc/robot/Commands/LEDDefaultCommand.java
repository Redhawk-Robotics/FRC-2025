// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.AlgaeHandler;
import frc.robot.subsystems.CoralHandler;
import frc.robot.Constants.Ports.CoralIntake;
import frc.robot.subsystems.AlgaeFloorIntake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDs;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LEDDefaultCommand extends Command {
  /** Creates a new LEDDefaultCommand. */
  Pivot m_pivot = new Pivot();
  Elevator m_elevator = new Elevator();
  Climber m_climber = new Climber();
  CoralHandler m_coralHandler = new CoralHandler();

  LEDs m_leds = new LEDs();
  public LEDDefaultCommand(LEDs m_leds) {
    this.m_leds = m_leds;
    addRequirements(m_leds, m_coralHandler, m_climber, m_elevator, m_pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_coralHandler.intakeFromStation() != null){
        if ( m_coralHandler.wasIntakeTriggered()) {
            m_leds.yellow();
        }   
    }
    if (m_coralHandler.spitItOut() != null){
        //TODO DOES THIS WORK?
        if (m_coralHandler.wasOuttakeTriggered()) {
            m_leds.greenBlink();
        }
    }
    if (m_climber.releaseClimbWinch() != null){
        m_leds.green();
    }
    //!!PROBLEM: HOW DO WE DO THIS?
   /*  if (m_elevator.useSpeed() = ){
        m_leds.violet();*/
    else {
        m_leds.redHawk();
    }
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

