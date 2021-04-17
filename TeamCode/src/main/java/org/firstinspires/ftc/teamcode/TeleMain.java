package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.commands.Com_Intake;
import org.firstinspires.ftc.teamcode.commands.Com_Outtake;
import org.firstinspires.ftc.teamcode.commands.Com_PickUp;
import org.firstinspires.ftc.teamcode.commands.Com_PutDown;
import org.firstinspires.ftc.teamcode.commands.Com_Shooter;
import org.firstinspires.ftc.teamcode.commands.drive.Com_Drive;
import org.firstinspires.ftc.teamcode.commands.rr.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.rr.TurnCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveSystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;
import org.firstinspires.ftc.teamcode.util.TimedAction;

@TeleOp(name="KekW")
public class TeleMain extends CommandOpMode {

    //Servos and Motors
    private Motor fL, fR, bL, bR;
    private Motor flyWheel, intakeA, intakeB, arm;
    private SimpleServo flicker, grabber;

    //Subsystems
    private DriveSystem driveSystem;
    private MecanumDriveSubsystem drive;
    private ShooterSubsystem shooterSystem;
    private IntakeSubsystem intakeSystem;
    private WobbleSubsystem wobbleSystem;

    //Commands
    private Com_Drive driveCommand;
    private Com_Shooter shooterCommand;
    private Com_Intake intakeCommand;
    private Com_Outtake outtakeCommand;
    private Com_PickUp pickUpCommand;
    private Com_PutDown putDownCommand;
    private InstantCommand grabberCommand;
    private SequentialCommandGroup autoPowershotsCommand;

    //Extranious
    private GamepadEx m_driverOp;
//    private Button slowDrive;
    private RevIMU imu;
    private FtcDashboard dashboard;
    private TimedAction flickerAction;
    private VoltageSensor voltageSensor;
    public double mult = 1.0;
    public double shootMult = 1.0;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //Servos and Motors
        fL = new Motor(hardwareMap, "fL");
        fR = new Motor(hardwareMap, "fR");
        bL = new Motor(hardwareMap, "bL");
        bR = new Motor(hardwareMap, "bR");

        flyWheel = new Motor(hardwareMap, "shoot");
        flyWheel.resetEncoder();
        flyWheel.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeA = new Motor(hardwareMap, "intakeA");
        intakeB = new Motor(hardwareMap, "intakeB", Motor.GoBILDA.RPM_312);
        intakeB.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeB.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm = new Motor(hardwareMap, "wobble", Motor.GoBILDA.RPM_312);
        arm.motor.setDirection(DcMotor.Direction.REVERSE);
        arm.encoder = intakeB.encoder;  // cool feature, hope it works

        flicker = new SimpleServo(hardwareMap, "flicker", 0, 270);
        grabber = new SimpleServo(hardwareMap, "wobbleS", -90, 180);

        //imu
        imu = new RevIMU(hardwareMap);
        imu.init();

        //Controller
        m_driverOp = new GamepadEx(gamepad1);
        dashboard = FtcDashboard.getInstance();

        //FlickerAction
        flickerAction = new TimedAction(
                ()-> flicker.setPosition(0.37),
                ()-> flicker.setPosition(0.6),
                115,
                true
        );

        //I DEMAND LEDS >:(
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        //Subsystems and Commands
        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), false);
        fL.motor.setDirection(DcMotor.Direction.FORWARD);
        bL.motor.setDirection(DcMotor.Direction.FORWARD);

        fL.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        driveSystem = new DriveSystem(fL, fR, bL, bR);
        driveCommand = new Com_Drive(driveSystem, m_driverOp::getLeftX, m_driverOp::getLeftY, m_driverOp::getRightX);

        shooterSystem = new ShooterSubsystem(flyWheel, flicker, flickerAction, voltageSensor, ()->shootMult);
        shooterCommand = new Com_Shooter(shooterSystem);

        intakeSystem = new IntakeSubsystem(intakeA, intakeB);
        intakeCommand = new Com_Intake(intakeSystem);
        outtakeCommand = new Com_Outtake(intakeSystem);

        wobbleSystem = new WobbleSubsystem(arm, grabber);
        grabberCommand = new InstantCommand(()-> {
            if(wobbleSystem.isGrabbing())
                wobbleSystem.openGrabber();
            else
                wobbleSystem.closeGrabber();
            }, wobbleSystem);

        pickUpCommand = new Com_PickUp(wobbleSystem);
        putDownCommand = new Com_PutDown(wobbleSystem);

//       Old Method no longer necessary:
//        slowDrive = new GamepadButton(m_driverOp, GamepadKeys.Button.Y)
//                .toggleWhenPressed(()->mult = 0.5, ()->mult = 1.0);

        m_driverOp.getGamepadButton(GamepadKeys.Button.BACK)
                .toggleWhenPressed(
                        autoPowershotsCommand = new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            fL.motor.setDirection(DcMotor.Direction.REVERSE);
                            bL.motor.setDirection(DcMotor.Direction.REVERSE);

                            fL.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            bR.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            bL.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                            fL.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            bR.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            bL.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        }, driveSystem),
                        new InstantCommand(()->drive.setPoseEstimate(new Pose2d(63, -10, Math.toRadians(180)))),
                        new TrajectoryFollowerCommand(drive, drive.trajectoryBuilder(new Pose2d(63, -10, Math.toRadians(180)), true)
                                .lineToLinearHeading(new Pose2d(0.9, -15.0, Math.toRadians(180)))
                                .build()),
                        new InstantCommand(shooterSystem::flickPos).andThen(new WaitCommand(350)),
                        new TurnCommand(drive, Math.toRadians(-6.0))
                                .alongWith(new InstantCommand(shooterSystem::homePos), new WaitCommand(500)),
                        new InstantCommand(shooterSystem::flickPos).andThen(new WaitCommand(350)),
                        new TurnCommand(drive, Math.toRadians(-7.5))
                                .alongWith(new InstantCommand(shooterSystem::homePos), new WaitCommand(500)),
                        new InstantCommand(shooterSystem::flickPos).andThen(new WaitCommand(350)),
                        new InstantCommand(shooterSystem::homePos),
                        new InstantCommand(() -> {
                            fL.motor.setDirection(DcMotor.Direction.FORWARD);
                            bL.motor.setDirection(DcMotor.Direction.FORWARD);
                        })
                ), new InstantCommand(()->{
                        autoPowershotsCommand.cancel();
                        shooterSystem.homePos();
                        fL.motor.setDirection(DcMotor.Direction.FORWARD);
                        bL.motor.setDirection(DcMotor.Direction.FORWARD);
                }));

        m_driverOp.getGamepadButton(GamepadKeys.Button.A).whenHeld(shooterCommand);

        m_driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenHeld(intakeCommand);
        m_driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenHeld(outtakeCommand);

        m_driverOp.getGamepadButton(GamepadKeys.Button.X).whenPressed(grabberCommand);
        m_driverOp.getGamepadButton(GamepadKeys.Button.B).toggleWhenPressed(putDownCommand, pickUpCommand);

        m_driverOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .toggleWhenPressed(
                        new SequentialCommandGroup(
                                new InstantCommand(()->{shootMult = 0.55;})
                        ),
                        new SequentialCommandGroup(
                                new InstantCommand(()->{shootMult = 1.0;})

                        )
                );
        m_driverOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .toggleWhenPressed(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> flyWheel.setRunMode(Motor.RunMode.VelocityControl)),
                                new RunCommand(shooterSystem::shoot)
                        ),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> flyWheel.setRunMode(Motor.RunMode.RawPower)),
                                new RunCommand(shooterSystem::stop)
                        )
                );

        register(driveSystem);
        driveSystem.setDefaultCommand(driveCommand);
        schedule(new RunCommand(() -> {
            telemetry.addData("FlywheelSpeed", shootMult);
            telemetry.update();
        }));
    }
}
