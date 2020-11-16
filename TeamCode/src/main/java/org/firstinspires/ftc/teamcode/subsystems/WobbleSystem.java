package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.Timing;

import java.util.concurrent.TimeUnit;


public class WobbleSystem extends SubsystemBase {
    MotorEx pickyUppy;
    public WobbleSystem(MotorEx pickMeUpDaddy){
        pickyUppy = pickMeUpDaddy;
    }

    public void spinMeRightRoundBaby(){
        //TODO: testing also change those names
        pickyUppy.setTargetPosition(10);
        pickyUppy.set(0.1);
    }
}

