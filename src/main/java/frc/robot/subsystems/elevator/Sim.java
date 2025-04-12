package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

// import com.ctre.phoenix6.hardware.TalonFX;
// import com.revrobotics.sim.SparkMaxSim;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

// TODO implement a real elevator simulation
// https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/elevatorsimulation/subsystems/Elevator.java

// NOTE, from CTRE:
// > Multiple CAN buses using the CANivore API is not supported at this time. All CAN devices will
// appear on the same CAN bus. If you wish to run your robot code in simulation, ensure devices have
// unique IDs across CAN buses.
//
// So, when creating simulated hardware for this, use different CAN IDs
// since our drivetrain CAN IDs overlap with the Elevator CAN IDs

class Sim extends Elevator { // extends Elevator
    Sim() {
        System.out.println("Constructing simulated Elevator.");
        // DCMotor.getNEO(4);
        // new SparkMax(10, MotorType.kBrushless);
        // new SparkMaxSim(null, null);

        // https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/physics-sim.html
        // https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/simulation/simulation-intro.html#supported-devices

        DCMotor gearbox = DCMotor.getKrakenX60(3);
        // new TalonFX(60).getSimState();

        var sim = new ElevatorSim(gearbox, 0.04, Pounds.of(10).in(Kilograms), 0.01, 0,
                Feet.of(4).in(Meters), true, 0, 0.01, 0);
    }

    @Override
    public void configureMotors(double kG, double kS, double kP, double kI, double kD) {

    }

    @Override
    public void resetElevatorPosition() {

    }

    @Override
    public Angle getPosition() {
        return Rotations.of(0);
    }

    @Override
    public AngularVelocity getVelocity() {
        return RotationsPerSecond.of(0);
    }

    @Override
    public void setReference(double reference) {

    }

    @Override
    public void setSpeed(double speed) {

    }

    @Override
    public void useSpeed() {

    }

    @Override
    public void stopElevator() {

    }
}
