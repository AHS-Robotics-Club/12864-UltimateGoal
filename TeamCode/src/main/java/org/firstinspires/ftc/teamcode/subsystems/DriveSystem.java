package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveSystem extends SubsystemBase {
    private MecanumDrive drive;
    private Motor fL, bL, fR, bR;

    public DriveSystem(Motor frontL, Motor frontR, Motor backL, Motor backR){
        fL = frontL;
        fR = frontR;
        bL = backL;
        bR = backR;

        drive = new MecanumDrive(fL, fR, bL, bR);
    }
    //not in current use and yuou cant make me get rid of it
//    public DriveSystem(HardwareMap hMap, String fLName, String fRName, String bLName, String bRName){
//        this(new Motor(hMap, fLName), new Motor(hMap, fRName), new Motor(hMap, bLName), new Motor(hMap, bRName));
//    }

    //Strafe Speed, Forward Speed, and Turn Speed
    public void drive(double strfSpd, double fSpd, double trnSpd){
        drive.driveRobotCentric(-strfSpd, fSpd, -trnSpd);
    }
}
