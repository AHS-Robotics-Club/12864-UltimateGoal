package org.firstinspires.ftc.teamcode.subsystems.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.WobbleSystem;

public class Com_AutonLift extends CommandBase {
    private final WobbleSystem wobblySystem;
    private Motor motor;
    private ElapsedTime timer;
    public Com_AutonLift(WobbleSystem subby, ElapsedTime time){
        wobblySystem = subby;
        motor = wobblySystem.getMotor();

        //so no head?

        timer = time;

        addRequirements(subby);
    }

    @Override
    public void initialize(){
        timer.reset();
        motor.setPositionCoefficient(0.01);
        motor.setPositionTolerance(10);
        motor.setTargetPosition(308);
        motor.stopMotor();
    }
    @Override
    public void execute(){
        wobblySystem.autonUp();
    }
    @Override
    public void end(boolean interruptable){
        timer.reset();
        wobblySystem.motorStop();
    }
    @Override
    public boolean isFinished(){
        return motor.atTargetPosition() || timer.seconds() > 0.30;
    }
}
