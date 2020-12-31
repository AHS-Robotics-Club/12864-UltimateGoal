package org.firstinspires.ftc.teamcode.testingFolder.forJackson;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class ShooterSubsystem extends SubsystemBase {

    private Motor flyWheel;
    private SimpleServo pusher;

    public ShooterSubsystem(Motor flyWheel, SimpleServo pusher){
        this.flyWheel = flyWheel;

        flyWheel.setRunMode(Motor.RunMode.VelocityControl);
        //These coefficients are placeholders you will need to put your own in their place
        flyWheel.setVeloCoefficients(1, 0, 0);
        flyWheel.setFeedforwardCoefficients(0, 1);

        this.pusher = pusher;

    }

    public void flyWheelShoot(){
        flyWheel.set(1.0);
    }
    public void flyWheelStop(){
        flyWheel.stopMotor();
    }

    //Unused, but would be used to push the ring into the flywheel
    public void pushRing(){
        pusher.turnToAngle(80);
    }
    public void retract(){
        pusher.turnToAngle(0);
    }

}
