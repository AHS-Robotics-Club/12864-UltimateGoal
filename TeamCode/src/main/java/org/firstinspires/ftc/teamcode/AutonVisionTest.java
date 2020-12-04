package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.stolenVision.UGContourRingDetector;
import org.firstinspires.ftc.teamcode.stolenVision.UGContourRingPipeline;
import org.firstinspires.ftc.teamcode.subsystems.ContourVisionSystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSystem;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSystem;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_Contour;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_PutDown;
import org.openftc.easyopencv.OpenCvInternalCamera;


import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.BRAKE;

@Autonomous(name="VisionPoo")
public class AutonVisionTest extends CommandOpMode {
        private Motor fL, bL, fR, bR;
        private Motor wobble, test;
        private CRServo servo;
        private UGContourRingDetector ugContourRingDetector;
        private UGContourRingPipeline ugContourRingPipeline;
        private DriveSystem mecDrive;

        private ContourVisionSystem visionSystem;
        private Com_Contour visionCommand;

        private WobbleSystem wobbleSystem;
        private Com_PutDown putDown;
        private ElapsedTime time;
        private RevIMU imu;
        private VoltageSensor voltageSensor;

        @Override
        public void initialize() {
            fL = new Motor(hardwareMap, "fL");
            fR = new Motor(hardwareMap, "fR");
            bL = new Motor(hardwareMap, "bL");
            bR = new Motor(hardwareMap, "bR");
            //one of our motors is messed up so it has to be inverted woooooo
            bL.setInverted(true);

            fL.setZeroPowerBehavior(BRAKE);
            fR.setZeroPowerBehavior(BRAKE);
            bL.setZeroPowerBehavior(BRAKE);
            bR.setZeroPowerBehavior(BRAKE);

            wobble = new Motor(hardwareMap, "wobble");
            servo = new CRServo(hardwareMap, "servo");
            wobble.setZeroPowerBehavior(BRAKE);
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
            //named shot purely because im too lazy to change config
            test = new Motor(hardwareMap, "shot");
            ugContourRingDetector = new UGContourRingDetector(hardwareMap, OpenCvInternalCamera.CameraDirection.BACK, telemetry, true);
            UGContourRingDetector.PipelineConfiguration.setIS_PORTRAIT_MODE(false);
            ugContourRingDetector.init();

            imu = new RevIMU(hardwareMap);
            imu.init();


            time = new ElapsedTime();
            mecDrive = new DriveSystem(fL, fR, bL, bR);
            wobbleSystem = new WobbleSystem(servo, wobble, telemetry, this::isStopRequested);
            putDown = new Com_PutDown(wobbleSystem, time);
            visionSystem = new ContourVisionSystem(ugContourRingDetector, telemetry);
            visionCommand = new Com_Contour(visionSystem);

            register(mecDrive, new SubsystemBase(){
                @Override
                public void periodic() {
                    telemetry.addData("rings", visionSystem.getStackSize());
                    telemetry.update();
                }
            });


        }
}
