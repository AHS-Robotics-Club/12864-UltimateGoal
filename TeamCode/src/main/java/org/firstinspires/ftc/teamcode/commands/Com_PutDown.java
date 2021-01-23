package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;

public class Com_PutDown extends CommandBase {

    private Motor arm;
    private WobbleSubsystem wobbleSystem;

    public Com_PutDown(WobbleSubsystem subsystem){
        wobbleSystem = subsystem;
        arm = wobbleSystem.getMotor();

        addRequirements(wobbleSystem);
    }

    @Override
    public void initialize(){
        arm.setPositionCoefficient(0.005);
        arm.setPositionTolerance(10);
        arm.setTargetPosition(-470);
    }

    @Override
    public void execute(){
        wobbleSystem.armDown();
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
