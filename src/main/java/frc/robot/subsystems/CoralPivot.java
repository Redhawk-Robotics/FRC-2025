// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;
import frc.robot.Constants.Settings;

public class CoralPivot extends SubsystemBase {
  /** Creates a new CoralPivot. */

  //Declare Motors here
  SparkMax leftMotor;
  SparkMax rightMotor;

  public CoralPivot() {
    // TODO GET VALUES FOR THESE
    this.leftMotor = new SparkMax(Ports.CoralIntake.PIVOT_LEFT, Settings.Pivot.LEFT_PIVOT_MOTORTYPE);
<<<<<<< Updated upstream
    this.rightMotor = new SparkMax(Ports.CoralIntake.PIVOT_RIGHT, Settings.Pivot.LEFT_PIVOT_MOTORTYPE);
=======
    this.rightMotor = new SparkMax(Ports.CoralIntake.PIVOT_RIGHT, Settings.Pivot.RIGHT_PIVOT_MOTORTYPE);
>>>>>>> Stashed changes
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  // TODO Uncomment lines 29-35 and implement routines for them.
//   public Command pivotUp() {}

//   public Comand pivotDown() {}

//   public Command wheelIntake() {}

//   public Command wheelOutake() {}

}
