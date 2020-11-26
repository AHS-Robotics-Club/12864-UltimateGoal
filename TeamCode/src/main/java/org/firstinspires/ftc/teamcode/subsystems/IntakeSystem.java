package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class IntakeSystem extends SubsystemBase {
    private Motor intakeMotor;
    private Telemetry telemetry;

    public IntakeSystem(Motor IntakeMotor) {
        intakeMotor = IntakeMotor;
    }

    public void suck() {
        intakeMotor.set(-0.9);
    }
    public void down(){
        intakeMotor.set(0.9);
    }
    public void stop() {
        intakeMotor.stopMotor();
    }

}
