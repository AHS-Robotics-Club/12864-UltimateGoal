package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.TimedAction;

public class ShooterSubsystem extends SubsystemBase {

    private Motor flywheel;
    private SimpleServo flicker;
    private TimedAction timedAction;
    private Telemetry telemetry;

    public ShooterSubsystem(Motor flywheel, SimpleServo flicker, TimedAction timedAction, Telemetry telemetry){
        this.flywheel = flywheel;

        this.flywheel.setRunMode(Motor.RunMode.VelocityControl);
        this.flywheel.setVeloCoefficients(1.2, 0, 0.07);
        this.flywheel.setFeedforwardCoefficients(0, 1.1);

        this.flicker = flicker;
        this.timedAction = timedAction;
        this.telemetry = telemetry;
    }

    public void shoot(){
        flywheel.set(1.0);
    }

    public void stop(){
        flywheel.stopMotor();
    }

    public void flick(){
        timedAction.run();
    }

    public void flickReset(){
        if (!timedAction.running())
            timedAction.reset();
    }

    @Override
    public void periodic(){
        telemetry.addData("Shooter Velocity:", flywheel.getCorrectedVelocity());
        telemetry.update();
    }
}
