package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;

public class Com_EndAutoPickUp extends CommandBase {

    private Motor arm;
    private WobbleSubsystem wobbleSystem;

    public Com_EndAutoPickUp(WobbleSubsystem subsystem){
        wobbleSystem = subsystem;
        arm = wobbleSystem.getMotor();

        addRequirements(wobbleSystem);
    }

    @Override
    public void initialize(){
        arm.setPositionCoefficient(0.005);
        arm.setPositionTolerance(10);
        arm.setTargetPosition(-100);
    }

    @Override
    public void execute(){
        wobbleSystem.armUp();
    }

    @Override
    public void end(boolean interrupted){
        wobbleSystem.stopMotor();
    }

    @Override
    public boolean isFinished(){
        return arm.atTargetPosition();
    }

}