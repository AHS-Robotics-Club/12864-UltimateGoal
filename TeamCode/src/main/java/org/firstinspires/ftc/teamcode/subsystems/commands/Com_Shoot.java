package org.firstinspires.ftc.teamcode.subsystems.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSystem;

import java.util.function.DoubleSupplier;

public class Com_Shoot extends CommandBase{

    private final ShooterSystem shooterSystem;

    public Com_Shoot(ShooterSystem subby){
        shooterSystem = subby;
        addRequirements(subby);
    }
    @Override
    public void execute(){
        shooterSystem.shoot();
    }

    @Override
    public void cancel() {
        shooterSystem.stop(); /* or whatever the name of that private instance is */
        CommandScheduler.getInstance().cancel(this);
    }
}
