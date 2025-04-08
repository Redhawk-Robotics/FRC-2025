// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#
 * defining-commands
 */
public class PlayMusic extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final Orchestra orchestra = new Orchestra();
    private final String filename;

    /** Creates a new PlayMusic. */
    public PlayMusic(String filename, CommandSwerveDrivetrain drivetrain) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.drivetrain = drivetrain;
        this.filename = filename;
        this.addRequirements(drivetrain);
        this.setName("PlayMusic(" + filename + ")");
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        for (SwerveModule<TalonFX, TalonFX, CANcoder> module : drivetrain.getModules()) {
            this.orchestra.addInstrument(module.getDriveMotor());
            this.orchestra.addInstrument(module.getSteerMotor());
        }
        var status =
                this.orchestra.loadMusic(Filesystem.getDeployDirectory() + "/chrp/" + filename);
        if (!status.isOK()) {
            // log error
            DriverStation.reportError("PlayMusic could not loadMusic:\n"+status.toString(), true);
        } else {
            System.out.println(this.orchestra.play());
        }
        System.out.printf("Orchestra is playing: %b\n", this.orchestra.isPlaying());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("Orchestra interrupted -- stopping");
            this.orchestra.stop();
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        boolean done = !this.orchestra.isPlaying();
        if (done) {
            System.out.println("Orchestra done playing!");
            this.orchestra.stop();
            // do we need to release the orchestra?
        }
        return done;
    }
}
