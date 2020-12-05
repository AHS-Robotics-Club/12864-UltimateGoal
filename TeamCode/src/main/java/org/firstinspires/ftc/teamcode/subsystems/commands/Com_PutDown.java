package org.firstinspires.ftc.teamcode.subsystems.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.WobbleSystem;

public class Com_PutDown extends CommandBase {

    private final WobbleSystem wobblySystem;

    private  Motor motor;

    public Com_PutDown(WobbleSystem subby){
        wobblySystem = subby;
        motor = wobblySystem.getMotor();
        motor.setPositionCoefficient(-0.01);
        motor.setPositionTolerance(10);
        motor.setTargetPosition(-345);

        addRequirements(subby);
    }
    @Override
    public void initialize(){
        motor.stopMotor();
        motor.resetEncoder();
    }
    @Override
    public void execute(){
        motor.set(-0.15);
    }
    @Override
    public void end(boolean interruptable){
        motor.stopMotor();
    }
    @Override
    public boolean isFinished(){
        return motor.atTargetPosition();
    }
}

