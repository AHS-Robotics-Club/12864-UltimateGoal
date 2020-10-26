package org.firstinspires.ftc.teamcode.subsystems.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSystem;

public class Com_Shoot extends CommandBase{

    private final ShooterSystem shooterSystem;

    public Com_Shoot(ShooterSystem subby){
        shooterSystem = subby;
        addRequirements(shooterSystem);
    }
    @Override
    public void execute(){
        shooterSystem.shot();
    }
}
