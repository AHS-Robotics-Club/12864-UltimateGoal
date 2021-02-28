package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.groups.FourRing;
import org.firstinspires.ftc.teamcode.commands.groups.OneRing;
import org.firstinspires.ftc.teamcode.commands.groups.ZeroRing;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ContourVisionSystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
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
    private Motor fL, fR, bL, bR, arm, flyWheel, intakeB, intakeA;
    private SimpleServo flicker, grabber;

    //Subsystems
    private MecanumDriveSubsystem drive;
    private WobbleSubsystem wobble;
    private ShooterSubsystem shooterSystem;
    private IntakeSubsystem intakeSystem;

    //Vision
    private UGContourRingDetector ugContourRingDetector;
    private ContourVisionSystem visionSystem;

    //Extranious
    private TimedAction flickerAction;
    private ElapsedTime time;
    private VoltageSensor voltageSensor;
    public boolean powerShotMode = false;
    private VisionSystem.Size height;
    //Poses

    //Trajectories

    @Override
    public void initialize() {
//        grabber = new SimpleServo(hardwareMap, "wobbleS", 0, 270);
//        grabber.setInverted(true);
//        grabber.setPosition(1);
        arm = new Motor(hardwareMap, "wobble", Motor.GoBILDA.RPM_312);
        intakeB = new Motor(hardwareMap, "intakeB", Motor.GoBILDA.RPM_312);
        intakeB.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeB.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeA = new Motor(hardwareMap, "intakeA", Motor.GoBILDA.RPM_1150);


        arm.motor.setDirection(DcMotor.Direction.REVERSE);
        arm.encoder = intakeB.encoder;
        grabber = new SimpleServo(hardwareMap, "wobbleS", -90, 180);
        time = new ElapsedTime();

        flyWheel = new Motor(hardwareMap, "shoot");
        flicker = new SimpleServo(hardwareMap, "flicker", 0, 270);

        flickerAction = new TimedAction(
                ()-> flicker.setPosition(0.90),
                ()-> flicker.setPosition(0.55),
                350,
                true
        );

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        shooterSystem = new ShooterSubsystem(flyWheel, flicker, flickerAction, voltageSensor);

        intakeSystem = new IntakeSubsystem(intakeA, intakeB);

        ugContourRingDetector = new UGContourRingDetector(hardwareMap, "poopcam", telemetry, true);
        ugContourRingDetector.init();
        visionSystem = new ContourVisionSystem(ugContourRingDetector, telemetry);

        arm.resetEncoder();

        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), false);
        wobble = new WobbleSubsystem(arm, grabber);

        wobble.closeGrabber();
        SequentialCommandGroup autonomous = new SequentialCommandGroup(
//                new WaitUntilCommand(this::isStarted),  Jacksons favorite line of code
                new SelectCommand(new HashMap<Object, Command>() {{
                        put(VisionSystem.Size.ZERO, (new ZeroRing(drive, wobble, shooterSystem)));
                        put(VisionSystem.Size.ONE, (new OneRing(drive, wobble, shooterSystem)));
                        put(VisionSystem.Size.FOUR, (new FourRing(drive, wobble, shooterSystem)));
                    }},()-> height)
        );
//        FtcDashboard.getInstance().startCameraStream(ugContourRingDetector.getCamera(), 30);

        while(!isStarted() && !isStopRequested()){
            height = visionSystem.getStackSize();
        }
        if(isStopRequested()){
            return;
        }
        schedule(new RunCommand(shooterSystem::shoot), autonomous);
    }
}
