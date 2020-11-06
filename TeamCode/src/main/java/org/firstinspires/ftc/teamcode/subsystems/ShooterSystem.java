package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TeleOpMain;

import java.util.function.DoubleSupplier;

public class ShooterSystem extends SubsystemBase {
    private MotorEx shooterMotor;
    private Telemetry telemetry;
    private DoubleSupplier power;

    public ShooterSystem(MotorEx ShooterMotor, Telemetry telemetryIn, DoubleSupplier getPower){
        shooterMotor = ShooterMotor;
        telemetry = telemetryIn;
        power = getPower;
    }

    public void shoot(){
        shooterMotor.set(power.getAsDouble());
    }

    public void stop(){
        shooterMotor.set(0);
    }
}
