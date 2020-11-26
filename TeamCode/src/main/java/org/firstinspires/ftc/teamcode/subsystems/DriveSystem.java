package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveSystem extends SubsystemBase {
    private MecanumDrive drive;
    private Motor fL, bL, fR, bR;
    private RevIMU imu;
    private boolean fieldCentric = false;

    //Overloaded constructor this will just make it easier when I want to test robot and field centric
    //Also Jackson you best tell me if there is already something that does this for me
    public DriveSystem(Motor frontL, Motor frontR, Motor backL, Motor backR){
        fL = frontL;
        fR = frontR;
        bL = backL;
        bR = backR;
        fieldCentric = false;
        drive = new MecanumDrive(fL, fR, bL, bR);
    }
    public DriveSystem(Motor frontL, Motor frontR, Motor backL, Motor backR, RevIMU revimu){
        fL = frontL;
        fR = frontR;
        bL = backL;
        bR = backR;
        fieldCentric = true;
        imu = revimu;
        drive = new MecanumDrive(fL, fR, bL, bR);
    }
    //not in current use and you cant make me get rid of it
//    public DriveSystem(HardwareMap hMap, String fLName, String fRName, String bLName, String bRName){
//        this(new Motor(hMap, fLName), new Motor(hMap, fRName), new Motor(hMap, bLName), new Motor(hMap, bRName));
//    }

    //Strafe Speed, Forward Speed, and Turn Speed
    public void drive(double strfSpd, double fSpd, double trnSpd){
        if(!fieldCentric)
            drive.driveRobotCentric(-strfSpd, -fSpd, -trnSpd*0.85, true);
        else
            drive.driveFieldCentric(-strfSpd, -fSpd, -trnSpd, imu.getHeading(), true);
    }
    public void halt(){
        drive.stop();
    }
}
