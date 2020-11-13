package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class IntakeSystem extends SubsystemBase {
    private Motor intakeMotor;
    private Telemetry telemetry;
    private boolean intakeActive;

    public IntakeSystem(Motor IntakeMotor) {
        intakeMotor = IntakeMotor;
        intakeActive = true;
    }
    public boolean active() {
        return intakeActive;
    }

    public void toggle() {
        intakeActive = !intakeActive;
    }
    public void suck() {
        intakeMotor.set(0.9);
    }

    public void stop() {
        intakeMotor.stopMotor();
    }

}
