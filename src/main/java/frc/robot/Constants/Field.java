// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Field {
    public static final Field2d globalField = new Field2d();

    // does this work?
    // static {
    //     System.out.println("#####################");
    //     SmartDashboard.putData("Global Field", globalField);
    // }
}
