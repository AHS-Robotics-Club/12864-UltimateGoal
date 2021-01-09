package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class IntakeSubsystem extends SubsystemBase {

    private Motor intakeA;

    public IntakeSubsystem(Motor intakeA){
        this.intakeA = intakeA;
    }

    public void start(){
        intakeA.set(1.0);
    }
    public void stop(){
        intakeA.stopMotor();
    }
    public void outtake(){
        intakeA.set(-1.0);
    }
}
