package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public class RingPushSystem extends SubsystemBase {
    private ServoEx servoPush;
    private ElapsedTime timeyWimey;
    public RingPushSystem(ServoEx pushy) {
        servoPush = pushy;
        timeyWimey = new ElapsedTime();
    }

    public void push(){
        servoPush.turnToAngle(40);
    }
}
