package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.vision.UGRectDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.jacksonSama.shhhnopeaking.ElapsedWait;
import org.firstinspires.ftc.teamcode.stolenVision.UGContourRingDetector;
import org.firstinspires.ftc.teamcode.subsystems.ContourVisionSystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSystem;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSystem;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_Contour;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_PutDown;
import org.firstinspires.ftc.teamcode.subsystems.commands.drive.Com_DriveTime;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_Vision;
import org.firstinspires.ftc.teamcode.subsystems.commands.drive.Com_Rotate;
import org.firstinspires.ftc.teamcode.subsystems.commands.groups.GroupFour;
import org.firstinspires.ftc.teamcode.subsystems.commands.groups.GroupOne;
import org.firstinspires.ftc.teamcode.subsystems.commands.groups.GroupZero;
import org.firstinspires.ftc.teamcode.subsystems.commands.groups.GroupZero_Shoot;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.HashMap;

import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.BRAKE;

@Autonomous(name="Kanye Shoot")
public class AutonWithShooter extends CommandOpMode {
    private Motor fL, bL, fR, bR;
    private Motor wobble, shot;
    private CRServo servo;
    private UGContourRingDetector ugContourRingDetector;
    private DriveSystem mecDrive;
    public double pewr = 1.0;


    private ContourVisionSystem visionSystem;
    private Com_Contour visionCommand;

    private WobbleSystem wobbleSystem;
    private Com_PutDown putDown;

    private ShooterSystem shooterSystem;

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
        wobble.setRunMode(Motor.RunMode.PositionControl);
        wobble.setZeroPowerBehavior(BRAKE);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        //named shot purely because im too lazy to change config
        shot = new Motor(hardwareMap, "shot");
        ugContourRingDetector = new UGContourRingDetector(hardwareMap, OpenCvInternalCamera.CameraDirection.BACK, telemetry, true);
        ugContourRingDetector.init();
        imu = new RevIMU(hardwareMap);
        imu.init();


        time = new ElapsedTime();
        mecDrive = new DriveSystem(fL, fR, bL, bR);
        wobbleSystem = new WobbleSystem(servo, wobble, telemetry);
        putDown = new Com_PutDown(wobbleSystem, time);
        shooterSystem = new ShooterSystem(shot, telemetry, ()->pewr);
        visionSystem = new ContourVisionSystem(ugContourRingDetector, telemetry);
        visionCommand = new Com_Contour(visionSystem, time);

        register(mecDrive, new SubsystemBase(){
            @Override
            public void periodic() {
                telemetry.addData("imu heading", imu.getHeading());
                telemetry.addData("rings", visionSystem.getStackSize());
                telemetry.update();
            }
        });

        SequentialCommandGroup wobbleGoal = new SequentialCommandGroup(
//                new FunctionalCommand(
//                        () -> { return; }, wobbleSystem::spinMeRightRoundBaby,
//                        bool -> wobbleSystem.servoStop(), () -> true, wobbleSystem),
                new Com_DriveTime(mecDrive, 0D, -0.55, 0D, time, 0.29),
                visionCommand,
                new SelectCommand(new HashMap<Object, Command>() {{
                    put(VisionSystem.Size.ZERO, new ScheduleCommand(new GroupZero_Shoot(mecDrive, time, voltageSensor, imu, wobbleSystem, shooterSystem)));
                    put(VisionSystem.Size.ONE, new ScheduleCommand(new GroupOne(mecDrive, time, voltageSensor, wobbleSystem, imu)));
                    put(VisionSystem.Size.FOUR, new ScheduleCommand(new GroupFour(mecDrive, time, voltageSensor, imu, wobbleSystem)));
                }},visionSystem::getStackSize)
        );

        schedule(wobbleGoal);
    }
}
