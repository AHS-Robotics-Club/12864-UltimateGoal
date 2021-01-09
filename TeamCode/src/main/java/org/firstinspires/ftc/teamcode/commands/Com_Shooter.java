package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class Com_Shooter extends CommandBase {

    private ShooterSubsystem shooterSubsystem;

    public Com_Shooter(ShooterSubsystem subsystem){
        shooterSubsystem = subsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){
        shooterSubsystem.flickReset();
    }

    @Override
    public void execute(){
        shooterSubsystem.flick();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    public void stopShooter(){
        shooterSubsystem.stop();
    }
}
