package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public abstract class Elevator extends SubsystemBase {

    // singleton pattern
    private static final Elevator instance;
    static {
        if (Robot.isReal()) {
            instance = new Impl();
        } else {
            instance = new Sim();
        }
    }

    public static Elevator getInstance() {
        return instance;
    }

    // methods that the Impl and Sim have to implement independently

    public abstract void configureMotors(double kG, double kS, double kP, double kI, double kD);
    public abstract void resetElevatorPosition();
    public abstract Angle getPosition();
    public abstract AngularVelocity getVelocity();
    public abstract void setReference(double reference);
    public abstract void setSpeed(double speed);
    public abstract void useSpeed();
    public abstract void stopElevator();

    // methods with the same implementation for both Impl and Sim

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator/Position", this.getPosition().in(Rotations));
        SmartDashboard.putNumber("Elevator/Velocity", this.getVelocity().in(RotationsPerSecond));
    }
}
