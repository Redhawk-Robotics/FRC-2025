package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.Settings;

public abstract class Elevator extends SubsystemBase {

    public enum Mode {
        kManualSpeed, kManualPosition, kPosition;
    }

    Mode mControlMode = Mode.kManualSpeed;
    private DoubleSupplier mInput = () -> 0.;
    double mSetPoint;

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

    // methods with the same implementation for both Impl and Sim

    public void stopElevator() {
        this.useControlMode(Mode.kManualSpeed, 0, () -> 0.);
    }

    public void useControlMode(Mode mode, double initialState, DoubleSupplier input) {
        this.mControlMode = mode;
        this.mSetPoint = initialState;
        if (input != null) {
            this.mInput = input;
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator/setpoint-input", this.mInput.getAsDouble());
        switch (this.mControlMode) {
            case kManualSpeed:
                // interpret the input func as directly setting the speed
                this.mSetPoint = MathUtil.clamp(this.mInput.getAsDouble(), -1., 1.);
                break;
            case kManualPosition:
                // interpet the input func as an addition to the setpoint
                this.mSetPoint = MathUtil.clamp(this.mSetPoint + this.mInput.getAsDouble(),
                        Settings.Positioner.minElevatorPosition,
                        Settings.Positioner.maxElevatorPosition);
                break;
            case kPosition:
                // interpet the input func as directly setting the position setpoint
                this.mSetPoint = MathUtil.clamp(this.mInput.getAsDouble(),
                        Settings.Positioner.minElevatorPosition,
                        Settings.Positioner.maxElevatorPosition);
                break;
            default:
                break;
        }

        SmartDashboard.putNumber("Elevator/Position", this.getPosition().in(Rotations));
        SmartDashboard.putNumber("Elevator/Velocity", this.getVelocity().in(RotationsPerSecond));
        SmartDashboard.putString("Elevator/Control-Mode", this.mControlMode.toString());
        SmartDashboard.putNumber("Elevator/Set-Point", this.mSetPoint);
    }
}
