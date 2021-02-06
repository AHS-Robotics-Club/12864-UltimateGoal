package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.groups.FourRing;
import org.firstinspires.ftc.teamcode.commands.groups.ZeroRing;
import org.firstinspires.ftc.teamcode.commands.groups.OneRing;
import org.firstinspires.ftc.teamcode.commands.vision.Com_Contour;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ContourVisionSystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.UGContourRingDetector;
import org.firstinspires.ftc.teamcode.subsystems.VisionSystem;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;
import org.firstinspires.ftc.teamcode.util.TimedAction;

import java.util.HashMap;

@Autonomous(name="PogU")
public class AutonMain extends CommandOpMode {
    //Servos and Motors
    private Motor fL, fR, bL, bR, arm, flyWheel;
    private SimpleServo flicker, grabber;

    //Subsystems
    private MecanumDriveSubsystem drive;
    private WobbleSubsystem wobble;
    private ShooterSubsystem shooterSystem;

    //Vision
    private UGContourRingDetector ugContourRingDetector;
    private ContourVisionSystem visionSystem;
    private Com_Contour visionCommand;

    //Extranious
    private TimedAction flickerAction;
    private ElapsedTime time;
    private VoltageSensor voltageSensor;
    public boolean powerShotMode = false;
    //Poses

    //Trajectories

    @Override
    public void initialize() {
//        grabber = new SimpleServo(hardwareMap, "wobbleS", 0, 270);
//        grabber.setInverted(true);
//        grabber.setPosition(1);
        arm = new Motor(hardwareMap, "wobble", Motor.GoBILDA.RPM_312);
        arm.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        grabber = new SimpleServo(hardwareMap, "wobbleS", -90, 180);
        time = new ElapsedTime();

        flyWheel = new Motor(hardwareMap, "shoot");
        flicker = new SimpleServo(hardwareMap, "flicker", 0, 270);

        flickerAction = new TimedAction(
                ()-> flicker.setPosition(0.5),
                ()-> flicker.setPosition(0.27),
                600,
                true
        );
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        shooterSystem = new ShooterSubsystem(flyWheel, flicker, flickerAction, voltageSensor);

        ugContourRingDetector = new UGContourRingDetector(hardwareMap, "poopcam", telemetry, true);
        ugContourRingDetector.init();
        visionSystem = new ContourVisionSystem(ugContourRingDetector, telemetry);
        visionCommand = new Com_Contour(visionSystem, time);

        arm.resetEncoder();

        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), false);
        wobble = new WobbleSubsystem(arm, grabber);

        wobble.closeGrabber();

        SequentialCommandGroup autonomous = new SequentialCommandGroup(
//                new WaitUntilCommand(this::isStarted),  Jacksons favorite line of code
                visionCommand,
                new SelectCommand(new HashMap<Object, Command>() {{
                        put(VisionSystem.Size.ZERO, (new ZeroRing(drive, wobble, shooterSystem)));
                        put(VisionSystem.Size.ONE, (new OneRing(drive, wobble, shooterSystem)));
                        put(VisionSystem.Size.FOUR, (new FourRing(drive, wobble, shooterSystem)));
                    }},visionSystem::getStackSize)
        );
        schedule(new RunCommand(shooterSystem::shoot), autonomous);
    }
}
