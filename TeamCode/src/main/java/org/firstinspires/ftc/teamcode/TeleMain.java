package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.Com_Intake;
import org.firstinspires.ftc.teamcode.commands.Com_Outtake;
import org.firstinspires.ftc.teamcode.commands.Com_PickUp;
import org.firstinspires.ftc.teamcode.commands.Com_PutDown;
import org.firstinspires.ftc.teamcode.commands.Com_Shooter;
import org.firstinspires.ftc.teamcode.commands.drive.Com_Drive;
import org.firstinspires.ftc.teamcode.commands.groups.SequentialShooter;
import org.firstinspires.ftc.teamcode.subsystems.DriveSystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;
import org.firstinspires.ftc.teamcode.util.TimedAction;

@TeleOp(name="KekW")
public class TeleMain extends CommandOpMode {
    //Servos and Motors
    private Motor fL, fR, bL, bR;
    private Motor flyWheel, intakeA, arm;
    private SimpleServo flicker, grabber;

    //Subsystems
    private DriveSystem driveSystem;
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
    private SequentialShooter shootCommandGroup;
    private InstantCommand grabberCommand;
    private InstantCommand runFlyWheelCommand;

    //Extranious
    private GamepadEx m_driverOp;
//    private Button slowDrive;
    private RevIMU imu;
    private FtcDashboard dashboard;
    private TimedAction flickerAction;
    public double mult = 1.0;

    @Override
    public void initialize() {
        //Servos and Motors
        fL = new Motor(hardwareMap, "fL");
        fR = new Motor(hardwareMap, "fR");
        bL = new Motor(hardwareMap, "bL");
        bR = new Motor(hardwareMap, "bR");

        flyWheel = new Motor(hardwareMap, "shoot");
        intakeA = new Motor(hardwareMap, "intakeA");
        arm = new Motor(hardwareMap, "wobble", Motor.GoBILDA.RPM_312);

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
                ()-> flicker.setPosition(0.5),
                ()-> flicker.setPosition(0.27),
                600,
                true
        );

        //Subsystems and Commands
        driveSystem = new DriveSystem(fL, fR, bL, bR);
        driveCommand = new Com_Drive(driveSystem, m_driverOp::getLeftX, m_driverOp::getLeftY,
                m_driverOp::getRightX, ()->mult);

        shooterSystem = new ShooterSubsystem(flyWheel, flicker, flickerAction, telemetry);
        shooterCommand = new Com_Shooter(shooterSystem);
        runFlyWheelCommand = new InstantCommand(shooterSystem::shoot, shooterSystem);
        shootCommandGroup = new SequentialShooter(runFlyWheelCommand,
                new WaitCommand(1500), shooterCommand);

        intakeSystem = new IntakeSubsystem(intakeA);
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

        m_driverOp.getGamepadButton(GamepadKeys.Button.Y)
                .toggleWhenPressed(()->mult = 0.5, ()->mult = 1.0);

        m_driverOp.getGamepadButton(GamepadKeys.Button.A).whenHeld(shootCommandGroup);

        m_driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenHeld(intakeCommand);
        m_driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenHeld(outtakeCommand);

        m_driverOp.getGamepadButton(GamepadKeys.Button.X).whenPressed(grabberCommand);
        m_driverOp.getGamepadButton(GamepadKeys.Button.B).toggleWhenPressed(putDownCommand, pickUpCommand);
        
        register(driveSystem);
        driveSystem.setDefaultCommand(driveCommand);
    }
}
