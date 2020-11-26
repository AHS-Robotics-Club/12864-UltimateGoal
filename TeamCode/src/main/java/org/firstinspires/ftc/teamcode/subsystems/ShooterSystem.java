package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;

public class ShooterSystem extends SubsystemBase {
    private Motor shooterMotor;
    private Telemetry telemetry;
    private DoubleSupplier power;
    private boolean shooterActive;

    public ShooterSystem(Motor ShooterMotor, Telemetry telemetryIn, DoubleSupplier getPower){
        shooterMotor = ShooterMotor;
        telemetry = telemetryIn;
        power = getPower;
        shooterActive = true;

    }

    public void shoot(){
        shooterMotor.set(power.getAsDouble());
        shooterActive = true;
    }

    public void stop(){
        shooterMotor.stopMotor();
        shooterActive = false;
    }

    @Override
    public void periodic() {
        telemetry.addData("Shooter power", power.getAsDouble());
        telemetry.addData("Shooter active", shooterActive);
        telemetry.update();
    }
}
