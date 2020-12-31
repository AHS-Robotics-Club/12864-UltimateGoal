package org.firstinspires.ftc.teamcode.testingFolder.forJackson;

import com.arcrobotics.ftclib.command.CommandBase;

public class FlyWheelCommand extends CommandBase {

    private ShooterSubsystem shooterSubsystem;

    public FlyWheelCommand(ShooterSubsystem subsystem){
        shooterSubsystem = subsystem;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute(){
        shooterSubsystem.flyWheelShoot();
    }

    @Override
    public void end(boolean interrupted){
        shooterSubsystem.flyWheelStop();
    }
}
