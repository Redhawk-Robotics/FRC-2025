package frc.robot.subsystems.elevator;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

class Sim { // extends Elevator
    Sim() {
        DCMotor.getNEO(4);
        new SparkMax(10, MotorType.kBrushless);
        new SparkMaxSim(null, null);
        // var sim = new ElevatorSim();
    }
}
