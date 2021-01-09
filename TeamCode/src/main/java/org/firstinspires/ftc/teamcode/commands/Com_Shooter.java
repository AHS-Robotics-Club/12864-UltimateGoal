package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class Com_Shooter extends CommandBase {

    private ShooterSubsystem shooterSubsystem;

    public Com_Shooter(ShooterSubsystem subsystem){
        shooterSubsystem = subsystem;
        addRequirements(shooterSubsystem);
    }

    public void execute(){
        shooterSubsystem.shoot();
    }

    public void end(boolean interrupted) {
        shooterSubsystem.stop();
    }
}
