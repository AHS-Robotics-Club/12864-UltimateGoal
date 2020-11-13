package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.Timing;

import java.util.concurrent.TimeUnit;


public class WobbleSystem extends SubsystemBase {
    MotorEx rundhi;
    public WobbleSystem(MotorEx pickMeUpDaddy){
        rundhi = pickMeUpDaddy;
    }

    public void spinMeRightRoundBaby(){
        //TODO: testing also change those names
        rundhi.setTargetPosition(10);
        rundhi.set(0.1);
    }
}

