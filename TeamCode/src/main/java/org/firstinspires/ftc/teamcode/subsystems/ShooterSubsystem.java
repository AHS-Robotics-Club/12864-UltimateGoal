package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShooterSubsystem extends SubsystemBase {

    private Motor flywheel;
    private Telemetry telemetry;

    public ShooterSubsystem(Motor flywheel, Telemetry telemetry){
        this.flywheel = flywheel;

        this.flywheel.setRunMode(Motor.RunMode.VelocityControl);
        this.flywheel.setVeloCoefficients(1.2, 0, 0.07);
        this.flywheel.setFeedforwardCoefficients(0, 1.1);

        this.telemetry = telemetry;
    }

    public void shoot(){
        flywheel.set(1.0);
    }

    public void stop(){
        flywheel.stopMotor();
    }

    @Override
    public void periodic(){
        telemetry.addData("Shooter Velocity:", flywheel.getCorrectedVelocity());
        telemetry.update();
    }
}
