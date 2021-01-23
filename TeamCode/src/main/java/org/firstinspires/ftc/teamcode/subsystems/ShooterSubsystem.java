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
        this.flywheel.setVeloCoefficients(1.1, 0, 0.05);
        this.flywheel.setFeedforwardCoefficients(0, 1.0);

        this.flicker = flicker;
        this.timedAction = timedAction;
        this.telemetry = telemetry;
    }

    public boolean isRunning() {
        return timedAction.running();
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

    public void resetEncoder(){
        flywheel.resetEncoder();
    }

    public void flickReset(){
        if (!timedAction.running())
            timedAction.reset();
    }
    public void homePos(){
        flicker.setPosition(0.27);
    }

    public void setRunMode(Motor.RunMode runMode){
        flywheel.setRunMode(runMode);
    }
}
