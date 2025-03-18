// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;
import frc.robot.Constants.Settings;
import com.revrobotics.spark.SparkMax;

public class CoralHandler extends SubsystemBase {
    /** Creates a new CoralHandler. */
    private final SparkMax coralIntakeMotor;
    /*//sensor can reliably detect gamepiece when it is fully inside the intake. 
    End of intake path/critical transfer point*/
    private final DigitalInput entranceSensor;
    //serve as flags that remember whether or not the sensor was triggered
    private Boolean isEmpty;
    private Boolean triggeredByIntake;


    public CoralHandler() {
        // TODO
        this.coralIntakeMotor = new SparkMax(Ports.CoralIntake.WHEEL_INTAKE, Settings.CoralHandler.CORAL_INTAKE_MOTORTYPE);
        //The valid Analog ports are 0-3
        this.entranceSensor = new DigitalInput(1);
    }

    //Please check over this entire block of sensor code
    public boolean intakeSensor() {
        return ! entranceSensor.get();
        }

    public boolean outtakeSensor() {
        return entranceSensor.get();
        }

    public void triggeredSensor(){
        if (entranceSensor.get() == false){
             triggeredByIntake = true;
             isEmpty = false;
             /*We may have to reset this boolean at some point in our code, 
             so that it can detect the next time a game piece passes through the sensor. I think...*/
        } 

        if (triggeredByIntake == true||isEmpty ==false) {
            System.out.println("Intake is full");
            stop();
        }
    }

    public void isEmpty(){
        if (entranceSensor.get() == true){
            triggeredByIntake = false;
            isEmpty = true;
        }
        
        if (isEmpty == true||triggeredByIntake==false) {
            System.out.println("Intake is empty");
        }
    }

    

    public Command intakeFromStation() {
        return this.runOnce(() -> {
            // TODO turn on the motor to intake
            coralIntakeMotor.set(1);
            DriverStation.reportWarning("I am running!", Thread.currentThread().getStackTrace());
        });
    }

    public Command spitItOut() {
        return this.runOnce(() -> {
            // TODO turn on the motor to outtake     
            coralIntakeMotor.set(-.7);
            DriverStation.reportWarning("Please implement me!", Thread.currentThread().getStackTrace());
        });
    }

    public Command stop() {
        return this.runOnce(() -> {
            coralIntakeMotor.set(0);
            DriverStation.reportWarning("Please implement me!", Thread.currentThread().getStackTrace());
        });
    }

    //IT WILL BE ASSUMED THAT A CORAL IS INSIDE UNTIL TRIGGERDBYINTAKE IS FALSE
  

    public Boolean wasIntakeTriggered() {
        return triggeredByIntake;
    }

    public Boolean wasOuttakeTriggered() {
        return isEmpty;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Coral intake motor current",coralIntakeMotor.getOutputCurrent());
        SmartDashboard.putNumber("Pivot/Motor Output Speed", coralIntakeMotor.get());
        SmartDashboard.putBoolean("Intake Sensor", intakeSensor());
        SmartDashboard.putBoolean("Intake is Empty", outtakeSensor());
    }

}
