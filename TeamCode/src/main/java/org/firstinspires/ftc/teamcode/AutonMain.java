package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.vision.UGContourRingDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.vision.Com_Contour;
import org.firstinspires.ftc.teamcode.subsystems.ContourVisionSystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSystem;

@Autonomous(name="PogU")
public class AutonMain extends CommandOpMode {
    //Servos and Motors
    private Motor fL, fR, bL, bR;

    //Subsystems
    private DriveSystem driveSystem;

    //Vision
    private UGContourRingDetector ugContourRingDetector;
    private ContourVisionSystem visionSystem;
    private Com_Contour visionCommand;

    //Extranious
    private ElapsedTime time;

    @Override
    public void initialize() {
//        fL = new Motor(hardwareMap, "fL");
//        fR = new Motor(hardwareMap, "fR");
//        bL = new Motor(hardwareMap, "bL");
//        bR = new Motor(hardwareMap, "bR");

        ugContourRingDetector = new UGContourRingDetector(hardwareMap, "poopcam", telemetry, true);
        ugContourRingDetector.init();
        visionSystem = new ContourVisionSystem(ugContourRingDetector, telemetry);
        visionCommand = new Com_Contour(visionSystem, time);

        FtcDashboard.getInstance().startCameraStream(ugContourRingDetector.getCamera(), 30);

//        driveSystem = new DriveSystem(fL, fR, bL, bR);
    }
}
