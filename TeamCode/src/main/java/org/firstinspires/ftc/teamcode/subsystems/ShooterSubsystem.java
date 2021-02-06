package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.TimedAction;

import java.util.function.BooleanSupplier;

@Config
public class ShooterSubsystem extends SubsystemBase {

    private Motor flywheel;
    private SimpleServo flicker;
    private TimedAction timedAction;
    public static double kP = 1.2, kI = 0.0, kD = 0.05;
    public static double kS = 0.0, kV = 1.2;

    public ShooterSubsystem(Motor flywheel, SimpleServo flicker, TimedAction timedAction,
                            VoltageSensor voltageSensor){
        this.flywheel = flywheel;

        this.flywheel.setRunMode(Motor.RunMode.VelocityControl);
        this.flywheel.setVeloCoefficients(kP, kI, kD);
        this.flywheel.setFeedforwardCoefficients(kS, kV * 12 / voltageSensor.getVoltage());

        this.flicker = flicker;
        this.timedAction = timedAction;
    }

    public boolean isRunning() {
        return timedAction.running();
    }

    public void shoot(){
            flywheel.set(1.0);
    }

    public void stop(){
        flywheel.setRunMode(Motor.RunMode.RawPower);
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
