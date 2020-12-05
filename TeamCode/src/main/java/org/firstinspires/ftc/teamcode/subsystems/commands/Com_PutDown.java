package org.firstinspires.ftc.teamcode.subsystems.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSystem;

public class Com_PutDown extends CommandBase {

    private final WobbleSystem wobblySystem;

    private Motor motor;
    private Telemetry telemetry;
    private ElapsedTime timer;

    public Com_PutDown(WobbleSystem subby, ElapsedTime timey){
        wobblySystem = subby;
        motor = wobblySystem.getMotor();
        motor.setPositionCoefficient(0.01);
        motor.setPositionTolerance(10);
        motor.setTargetPosition(-330);
        telemetry = wobblySystem.getTele();

        timer = timey;

        addRequirements(subby);
    }
    @Override
    public void initialize(){
        timer.reset();
        motor.stopMotor();
        telemetry.addData("awdwad", "awdaw");
        telemetry.update();
    }
    @Override
    public void execute(){
        wobblySystem.armDown();
    }
    @Override
    public void end(boolean interruptable){
        timer.reset();
        wobblySystem.motorStop();
    }
    @Override
    public boolean isFinished(){
        return motor.atTargetPosition() || timer.seconds() > 0.4;
    }
}
