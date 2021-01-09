package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
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

import org.firstinspires.ftc.teamcode.commands.Com_Intake;
import org.firstinspires.ftc.teamcode.commands.Com_Outtake;
import org.firstinspires.ftc.teamcode.commands.Com_Shooter;
import org.firstinspires.ftc.teamcode.subsystems.DriveSystem;
import org.firstinspires.ftc.teamcode.commands.drive.Com_Drive;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.TimedAction;

@TeleOp(name="KekW")
public class TeleMain extends CommandOpMode {
    //Servos and Motors
    private Motor fL, fR, bL, bR;
    private Motor flyWheel, intakeA;
    private SimpleServo flicker;

    //Subsystems
    private DriveSystem driveSystem;
    private ShooterSubsystem shooterSystem;
    private IntakeSubsystem intakeSystem;

    //Commands
    private Com_Drive driveCommand;
    private Com_Shooter shooterCommand;
    private Com_Intake intakeCommand;
    private Com_Outtake outtakeCommand;
    private SequentialCommandGroup shootCommandGroup;
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

        flicker = new SimpleServo(hardwareMap, "flicker", 0, 270);

        //imu
        imu = new RevIMU(hardwareMap);
        imu.init();

        //Controller
        m_driverOp = new GamepadEx(gamepad1);
        dashboard = FtcDashboard.getInstance();

        //FlickerAction
        flickerAction = new TimedAction(
                ()-> flicker.setPosition(0.27),
                ()-> flicker.setPosition(0.5),
                500,
                true
        );

        //Subsystems and Commands
        driveSystem = new DriveSystem(fL, fR, bL, bR);
        driveCommand = new Com_Drive(driveSystem, m_driverOp::getLeftX, m_driverOp::getLeftY, m_driverOp::getRightX, ()->mult);

        shooterSystem = new ShooterSubsystem(flyWheel, flicker, flickerAction, telemetry);
        shooterCommand = new Com_Shooter(shooterSystem);
        runFlyWheelCommand = new InstantCommand(()->flyWheel.set(1.0), shooterSystem);
        shootCommandGroup = new SequentialCommandGroup(runFlyWheelCommand,
                new WaitCommand(1000),shooterCommand);

        intakeSystem = new IntakeSubsystem(intakeA);
        intakeCommand = new Com_Intake(intakeSystem);
        outtakeCommand = new Com_Outtake(intakeSystem);

//       Old Method no longer necessary:
//        slowDrive = new GamepadButton(m_driverOp, GamepadKeys.Button.Y)
//                .toggleWhenPressed(()->mult = 0.5, ()->mult = 1.0);

        m_driverOp.getGamepadButton(GamepadKeys.Button.Y)
                .toggleWhenPressed(()->mult = 0.5, ()->mult = 1.0);

        m_driverOp.getGamepadButton(GamepadKeys.Button.A).toggleWhenActive(shootCommandGroup);

        m_driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).toggleWhenActive(intakeCommand);
        m_driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).toggleWhenActive(outtakeCommand);

        register(driveSystem);
        driveSystem.setDefaultCommand(driveCommand);
    }
}

