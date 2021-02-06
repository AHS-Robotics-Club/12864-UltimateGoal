package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class RapidFireCommand extends CommandBase {

    private ShooterSubsystem shooterSubsystem;
    private int numShots;

    public RapidFireCommand(ShooterSubsystem subsystem){
        shooterSubsystem = subsystem;
        numShots = 0;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.flickReset();
        numShots = 0;
        shooterSubsystem.setRunMode(Motor.RunMode.VelocityControl);
    }

    @Override
    public void execute() {
        if (!shooterSubsystem.isRunning()) {
            shooterSubsystem.flickReset();
            numShots++;
        }
        shooterSubsystem.flick();
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.homePos();
        shooterSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return numShots == 3;
    }
}
