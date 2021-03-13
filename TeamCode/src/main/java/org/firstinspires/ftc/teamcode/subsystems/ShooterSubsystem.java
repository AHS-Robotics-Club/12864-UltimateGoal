package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.TimedAction;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

@Config
public class ShooterSubsystem extends SubsystemBase {

    private Motor flywheel;
    private SimpleServo flicker;
    private TimedAction timedAction;
    public static double kP = 18.2, kI = 0.0, kD = 0.2;
    public static double kS = 0.0, kV = 1.4;
    public static DoubleSupplier shooterSpeed;

    public ShooterSubsystem(Motor flywheel, SimpleServo flicker, TimedAction timedAction,
                            VoltageSensor voltageSensor){
        this.flywheel = flywheel;

        this.flywheel.setRunMode(Motor.RunMode.VelocityControl);
        this.flywheel.setVeloCoefficients(kP, kI, kD);
        this.flywheel.setFeedforwardCoefficients(kS, kV * 12 / voltageSensor.getVoltage());

        this.flicker = flicker;
        this.timedAction = timedAction;

        shooterSpeed = ()-> 1.0;
    }

    public ShooterSubsystem(Motor flywheel, SimpleServo flicker, TimedAction timedAction,
                            VoltageSensor voltageSensor, DoubleSupplier shootSpd){
        this.flywheel = flywheel;

        this.flywheel.setRunMode(Motor.RunMode.VelocityControl);
        this.flywheel.setVeloCoefficients(kP, kI, kD);
        this.flywheel.setFeedforwardCoefficients(kS, kV * 12 / voltageSensor.getVoltage());

        this.flicker = flicker;
        this.timedAction = timedAction;

        shooterSpeed = shootSpd;
    }

    public boolean isRunning() {
        return timedAction.running();
    }

    public void shoot(){
            flywheel.set(shooterSpeed.getAsDouble());
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

    public void flickPos(){
        flicker.setPosition(0.9);
    }

    public void homePos(){
        flicker.setPosition(0.55);
    }

    public void setRunMode(Motor.RunMode runMode){
        flywheel.setRunMode(runMode);
    }
}
