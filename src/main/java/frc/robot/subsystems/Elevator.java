// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;
import frc.robot.Constants.Settings;
import javax.print.attribute.SetOfIntegerSyntax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.AlternateEncoderConfigAccessor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

// see:
// https://github.com/REVrobotics/REVLib-Examples/blob/9b4cd410b6cc7fa8ed96b324dd9ecf1b4a2bbfd5/Java/SPARK/Open%20Loop%20Arcade%20Drive/src/main/java/frc/robot/Robot.java

public class Elevator extends SubsystemBase {

    public enum Setpoint {
        kFeederStation,
        kL1,
        kL2,
        kL3,
        kL4;
    }

    // private final SparkMax RelativeEncoder;
    private final SparkMax firstMotor;
    private final SparkMax secondMotor;
    private final SparkMax thirdMotor;
    private final SparkMax fourthMotor;

    private final AlternateEncoderConfig throughBoreConfig;
    private final RelativeEncoder throughBoreEncoder;

    private final SparkClosedLoopController elevatorController;
    private double elevatorCurrentTarget = Settings.Elevator.HOME_SETPOINT;

    // https://docs.revrobotics.com/rev-crossover-products/sensors/tbe/application-examples#brushless-motors

    /** Creates a new Elevator Subsystem. */
    public Elevator() {

        // TODO make these IDs into constants
        // looking at the elevator with the motors in view
        this.firstMotor = new SparkMax(Ports.Elevator.kCAN_ID_TOP_RIGHT, MotorType.kBrushless); // top-right
        this.secondMotor = new SparkMax(Ports.Elevator.kCAN_ID_BOTTOM_RIGHT, MotorType.kBrushless); // bottom-right (**leader**)
        this.thirdMotor = new SparkMax(Ports.Elevator.kCAN_ID_TOP_LEFT, MotorType.kBrushless); // top-left
        this.fourthMotor = new SparkMax(Ports.Elevator.kCAN_ID_BOTTOM_LEFT, MotorType.kBrushless); // bottom-left

        this.throughBoreConfig = new AlternateEncoderConfig();
        /* Through bore is registered as alternate, but is interpreted as a relative encoder object. */
        this.throughBoreEncoder = secondMotor.getAlternateEncoder();
        this.elevatorController = secondMotor.getClosedLoopController();

        // TODO verify these config settings
        SparkMaxConfig globalConfig = new SparkMaxConfig();
        globalConfig.smartCurrentLimit(60).idleMode(IdleMode.kBrake);

        SparkMaxConfig firstMotorConfig = new SparkMaxConfig();
        SparkMaxConfig secondMotorConfig = new SparkMaxConfig();
        SparkMaxConfig thirdMotorConfig = new SparkMaxConfig();
        SparkMaxConfig fourthMotorConfig = new SparkMaxConfig();

        // TODO verify these config settings
        firstMotorConfig.apply(globalConfig).follow(secondMotor);

        secondMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
        .p(0)
        .i(0)
        .d(0)
        .outputRange(-0.8, 0.8)
        .velocityFF(0.2);  // TODO FIGURE OUT WHAT THIS IS

        secondMotorConfig.closedLoop.maxMotion
        .maxVelocity(10)
        .maxAcceleration(10);

        secondMotorConfig.apply(globalConfig);
        // TODO CONFIRM THIS
        

        thirdMotorConfig.apply(globalConfig).inverted(Settings.Elevator.TOP_LEFT_INVERT)
                .follow(secondMotor);
        fourthMotorConfig.apply(globalConfig).inverted(Settings.Elevator.BOTTOM_LEFT_INVERT)
                .follow(secondMotor);


        
        secondMotorConfig.alternateEncoder.apply(new
        AlternateEncoderConfig().countsPerRevolution(8192));

        firstMotor.configure(firstMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        secondMotor.configure(secondMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        thirdMotor.configure(thirdMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        fourthMotor.configure(fourthMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // Zero the encoders on init
        throughBoreEncoder.setPosition(0);

    }

    // && UPDATES SETPOINTS WITH SCHEDULER 
    /* This will be run periodically with the scheduler. */
    public void moveToSetPoint() {
        /*
         The elevatorCurrentTargetVariable is changed by the Command method setSetPointCommand, as
         this runs continously in periodic() method.
         */
        elevatorController.setReference(elevatorCurrentTarget, ControlType.kMAXMotionPositionControl);
    }

    // && CASE SWITCH COMMANDS FOR ELEVATOR SETPOINTS, REUSED IN COMMAND METHODS USED IN ROBOTCONTAINER
    public Command setSetpointCommand(Setpoint setpoint) {
        /*
         This returns a command that changes the setpoint of the elevator based on
         the parameters. More specifically, it takes the input and changes the variable 
         "elevatorCurrentTarget" to the target it needs to travel to. This is then later implemented
         in a closed PID loop.
         */
        return this.runOnce(
            () -> {
                switch(setpoint) {
                    case kFeederStation:
                        elevatorCurrentTarget = Settings.Elevator.FEEDERSTATION_SETPOINT;
                        break;
                    case kL1:
                        elevatorCurrentTarget = Settings.Elevator.L1_SETPOINT;
                        break;
                    case kL2:
                        elevatorCurrentTarget = Settings.Elevator.L2_SETPOINT;
                        break;
                    case kL3:
                        elevatorCurrentTarget =  Settings.Elevator.L3_SETPOINT;
                        break;
                    case kL4:
                        elevatorCurrentTarget = Settings.Elevator.L4_SETPOINT; 
                }
            });
    }

    public Command up() {
        // TODO invert this in testing if needed
        return this.applySpeed(0.5);    
    }

    public Command down() {
        // TODO invert this in testing if needed
        return this.applySpeed(-0.5);
    }

    public Command stop() {
        return this.applySpeed(0);
    }

    public Command applySpeed(double speed) {
        // SubsystemBase.runOnce implicitly requires `this` subsystem.
        return this.runOnce(() -> {
            this.secondMotor.set(speed);
        });
    }

    public Command Feeder() {
        return this.runOnce(() -> {
            /* This runs the elevator to the L1 position */
            // TODO TO BE TESTED
            DriverStation.reportWarning("Running to L1", null);
            setSetpointCommand(Setpoint.kFeederStation);
            
        });
    }

    public Command L1() {
        return this.runOnce(() -> {
            /* This runs the elevator to the L1 position */
            // TODO TO BE TESTED
            DriverStation.reportWarning("Running to L1", null);
            setSetpointCommand(Setpoint.kL1);
            
        });
    }

    public Command L2() {
        return this.runOnce(() -> {
            /* This runs the elevator to the L2 position */
            // TODO TO BE TESTED
            DriverStation.reportWarning("Running to L2", null);
            setSetpointCommand(Setpoint.kL2);
        });
    }

    public Command L3() {
        return this.runOnce(() -> {
            /* This runs the robot to the L3 Position */
            // TODO TO BE TESTED
            DriverStation.reportWarning("Running to L3", null);
            setSetpointCommand(Setpoint.kL3);
        });
    }

    public Command L4() {
        return this.runOnce(() -> {
            /* This runs the robot to the L4 Position */
            // TODO TO BE TESTED 
            DriverStation.reportWarning("Running to L4", null);
            setSetpointCommand(Setpoint.kL4);
        });
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Elevator Real Position", throughBoreEncoder.getPosition());
        moveToSetPoint();
    }
}
