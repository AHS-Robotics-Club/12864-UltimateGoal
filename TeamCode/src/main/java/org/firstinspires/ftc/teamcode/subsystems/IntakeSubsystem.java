package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class IntakeSubsystem extends SubsystemBase {

    private Motor intakeA;
    private Motor intakeB;

    public IntakeSubsystem(Motor intakeA, Motor intakeB){
        this.intakeA = intakeA;
        this.intakeB = intakeB;
    }

    public void start(){
        intakeA.set(1.0);
        intakeB.set(1.0);
    }
    public void stop(){
        intakeA.stopMotor();
        intakeB.stopMotor();
    }
    public void outtake(){
        intakeA.set(-1.0);
        intakeB.set(-1.0);
    }
}
