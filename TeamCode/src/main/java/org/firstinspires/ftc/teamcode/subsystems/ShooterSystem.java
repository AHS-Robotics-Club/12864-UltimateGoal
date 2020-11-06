package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.TeleOpMain;

import java.util.function.DoubleSupplier;

public class ShooterSystem extends SubsystemBase {
    private MotorEx Motor_S;
    private TeleOpMain teleop;
    public double shooterPower;

    public ShooterSystem(MotorEx ShooterMotor, TeleOpMain tele){
        Motor_S = ShooterMotor;
        teleop = tele;
    }

    public void adjust(){
        //These are just place holder values need real ones
        switch(teleop.pwrSelect) {
            case 1:
                shooterPower = 0.5;
                teleop.telemetry.addData("Shooter Power:", shooterPower);
                teleop.telemetry.update();
                break;
            case 2:
                shooterPower = 0.75;
                teleop.telemetry.addData("Shooter Power:", shooterPower);
                teleop.telemetry.update();
                break;
            case 3:
                shooterPower = 1;
                teleop.telemetry.addData("Shooter Power:", shooterPower);
                teleop.telemetry.update();
                break;
            default:
                shooterPower = 0;
                teleop.telemetry.addData("Shooter Power:", shooterPower);
                teleop.telemetry.update();
                break;
        }
    }

    public void shoot(){
        Motor_S.set(shooterPower);
    }

    public void stop(){
        Motor_S.set(0);
    }
}
