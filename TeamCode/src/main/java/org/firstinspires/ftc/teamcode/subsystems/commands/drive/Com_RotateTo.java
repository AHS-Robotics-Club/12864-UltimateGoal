package org.firstinspires.ftc.teamcode.subsystems.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.RevIMU;

import org.firstinspires.ftc.teamcode.subsystems.DriveSystem;

public class Com_RotateTo extends CommandBase {
    private final DriveSystem driveSystem;
    private final RevIMU imu;
    private PIDController pidController;
    private final double degrees;

    public Com_RotateTo(DriveSystem subby, RevIMU revIMU, double degreesIn){
        driveSystem = subby;
        imu = revIMU;
        degrees = degreesIn;
        pidController = new PIDController(0.0492,0.9, 0.007);
        addRequirements(subby);
    }
    @Override
    public void initialize(){
        pidController.setSetPoint(degrees);
        pidController.setTolerance(5);
        pidController.calculate(imu.getRotation2d().getDegrees());
    }
    @Override
    public void execute() {
        driveSystem.drive(0, 0, -pidController.calculate(imu.getRotation2d().getDegrees()));
    }
    @Override
    public void end(boolean interrupted){
        pidController.reset();
        driveSystem.halt();
    }
    @Override
    public boolean isFinished(){
        return pidController.atSetPoint();
    }
}

