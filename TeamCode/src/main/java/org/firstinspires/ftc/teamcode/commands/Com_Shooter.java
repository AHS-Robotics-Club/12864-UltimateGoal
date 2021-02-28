package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

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
        shooterSubsystem.flickReset();
        shooterSubsystem.flick();
    }
    
    @Override
    public void end(boolean interrupted){
        shooterSubsystem.homePos();
    }

    public void stopShooter(){
        shooterSubsystem.setRunMode(Motor.RunMode.RawPower);
        shooterSubsystem.stop();
    }

    public void returnHome(){
        shooterSubsystem.homePos();
    }
}
