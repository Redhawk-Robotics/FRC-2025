// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    public Command setSpeeds(double armSP, double rollerS) {
        return this.runOnce(() -> {
            this.m_arm.setSpeed(armSP);
            this.m_roller.setSpeed(rollerS);
        });
    }

    public void setArmAndRoller(Double armRef, Double rollerSpeed) {
        if (armRef != null) {
            this.m_arm.setRef(armRef);
        }
        if (rollerSpeed != null) {
            this.m_roller.setSpeed(rollerSpeed);
        }
    }

    public double getArmPosition() {
        return this.m_arm.getPosition();
    }

    public double getRollerSpeed() {
        return this.m_roller.getSpeed();
    }

    public void stopArm() {
        this.m_arm.setSpeed(0);
    }

    public void stopArmAndRoller() {
        this.m_arm.setSpeed(0);
        this.m_roller.setSpeed(0);
    }

    public void resetArmPosition() {
        System.out.printf("Resetting Algae Arm encoder position (%f) -> zero\n",
                this.getArmPosition());
        this.m_arm.setPosition(0.0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Spoiler/Arm/Position", this.m_arm.getPosition());
        SmartDashboard.putNumber("Spoiler/Arm/Velocity", this.m_arm.getVelocity());
    }
}
