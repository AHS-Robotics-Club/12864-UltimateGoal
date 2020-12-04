package org.firstinspires.ftc.teamcode.subsystems.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DriveSystem;

public class Com_DriveTime extends CommandBase {

    private final DriveSystem driveSystem;
    private final Double strafe, forward, turn;
    private final ElapsedTime timey;
    private final double timeLength;

    public Com_DriveTime(DriveSystem subsystem, double strafeSpeed,
                         double forwardSpeed, double turnSpeed, ElapsedTime time, double amnt){
        driveSystem = subsystem;
        strafe = strafeSpeed;
        forward = forwardSpeed;
        turn = turnSpeed;
        timey = time;
        timeLength = amnt;
    }

    @Override
    public void initialize(){
        timey.reset();
    }
    @Override
     public void execute(){
        driveSystem.drive(strafe, forward, turn);
    }
    @Override
    public void end(boolean interrupted){
        driveSystem.halt();

    }

    @Override
    public boolean isFinished(){
        return timey.seconds() >= timeLength;
    }
}
