package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.vision.UGRectDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DriveSystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSystem;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSystem;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_PutDown;
import org.firstinspires.ftc.teamcode.subsystems.commands.drive.Com_DriveTime;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_Vision;
import org.firstinspires.ftc.teamcode.subsystems.commands.drive.Com_Rotate;
import org.firstinspires.ftc.teamcode.subsystems.commands.groups.GroupFour;
import org.firstinspires.ftc.teamcode.subsystems.commands.groups.GroupOne;
import org.firstinspires.ftc.teamcode.subsystems.commands.groups.GroupZero;

import java.util.HashMap;

import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.BRAKE;

@Autonomous(name="Kanye North")
public class AutonomousKanye extends CommandOpMode {
    private Motor fL, bL, fR, bR;
    private Motor wobble, test;
    private SimpleServo servo;
    private UGRectDetector ugRectDetector;
    private DriveSystem mecDrive;

    private VisionSystem visionSystem;
    private Com_Vision visionCommand;

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
        servo = new SimpleServo(hardwareMap, "servo");
        wobble.setZeroPowerBehavior(BRAKE);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        //named shot purely because im too lazy to change config
        test = new Motor(hardwareMap, "shot");
        ugRectDetector = new UGRectDetector(hardwareMap);
        ugRectDetector.init();
        ugRectDetector.setTopRectangle(0.46, 0.45);
        ugRectDetector.setBottomRectangle(0.46, 0.39);
        ugRectDetector.setRectangleSize(10, 30);
        imu = new RevIMU(hardwareMap);
        imu.init();


        time = new ElapsedTime();
        mecDrive = new DriveSystem(fL, fR, bL, bR);
        wobbleSystem = new WobbleSystem(servo, wobble, telemetry);
        putDown = new Com_PutDown(wobbleSystem, time);
        visionSystem = new VisionSystem(ugRectDetector, telemetry);
        visionCommand = new Com_Vision(visionSystem);

                register(mecDrive, new SubsystemBase(){
            @Override
            public void periodic() {
                telemetry.addData("imu heading", imu.getHeading());
                telemetry.addData("rings", visionSystem.getStackSize());
                telemetry.update();
            }
        });

        SequentialCommandGroup wobbleGoal = new SequentialCommandGroup(
                visionCommand,
                new SelectCommand(new HashMap<Object, Command>() {{
                    put(VisionSystem.Size.ZERO, new ScheduleCommand(new GroupZero(mecDrive, time, voltageSensor, imu, wobbleSystem)));
                    put(VisionSystem.Size.ONE, new ScheduleCommand(new GroupOne(mecDrive, time, voltageSensor, wobbleSystem)));
                    put(VisionSystem.Size.FOUR, new ScheduleCommand(new GroupFour(mecDrive, time, voltageSensor, imu, wobbleSystem)));
                }},visionSystem::getStackSize)
        );

        schedule(wobbleGoal);
    }
}