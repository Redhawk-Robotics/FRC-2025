// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.AlgaeFloorIntakeComponents.AlgaeFloorIntakeArm;
import frc.robot.subsystems.AlgaeFloorIntakeComponents.AlgaeFloorIntakeRoller;

public class AlgaeFloorIntake extends SubsystemBase {
    private final AlgaeFloorIntakeArm m_arm;
    private final AlgaeFloorIntakeRoller m_roller;

    /** Creates a new AlgaeFloorIntake. */
    public AlgaeFloorIntake(AlgaeFloorIntakeArm arm, AlgaeFloorIntakeRoller roller) {
        this.m_arm = arm;
        this.m_roller = roller;
    }

    public Command setStuff(double armSP, double rollerS) {
        return this.runOnce(() -> {
            this.m_arm.setRef(armSP);
            this.m_roller.setSpeed(rollerS);
        });
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
