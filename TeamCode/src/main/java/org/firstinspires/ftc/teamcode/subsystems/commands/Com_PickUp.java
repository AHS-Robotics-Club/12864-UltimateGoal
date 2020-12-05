package org.firstinspires.ftc.teamcode.subsystems.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.WobbleSystem;

public class Com_PickUp extends CommandBase {
    private final WobbleSystem wobblySystem;
    private Motor motor;
    private ElapsedTime timer;
    public Com_PickUp(WobbleSystem subby, ElapsedTime time){
        wobblySystem = subby;
        motor = wobblySystem.getMotor();
        motor.setPositionCoefficient(0.01);
        motor.setPositionTolerance(10);
        motor.setTargetPosition(330);

        timer = time;

        addRequirements(subby);
    }

    @Override
    public void initialize(){
        motor.stopMotor();
        timer.reset();
    }
    @Override
    public void execute(){
        wobblySystem.armUp();
    }
    @Override
    public void end(boolean interruptable){
        wobblySystem.motorStop();
    }
    @Override
    public boolean isFinished(){
        return motor.atTargetPosition() || timer.seconds() > 0.4;
    }
}