package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterSystem extends SubsystemBase {
    private Motor kuunav;

    public ShooterSystem(Motor ShooterMotor){
        kuunav = ShooterMotor;
    }

    public void shot(){
        kuunav.set(1);
    }

    public void stoop(){
        kuunav.set(0);
    }
}
