package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.Com_Intake;
import org.firstinspires.ftc.teamcode.commands.Com_Outtake;
import org.firstinspires.ftc.teamcode.commands.Com_Shooter;
import org.firstinspires.ftc.teamcode.subsystems.DriveSystem;
import org.firstinspires.ftc.teamcode.commands.drive.Com_Drive;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@TeleOp(name="KekW")
public class TeleMain extends CommandOpMode {
    //Servos and Motors
    private Motor fL, fR, bL, bR;
    private Motor flyWheel, intakeA;

    //Subsystems
    private DriveSystem driveSystem;
    private ShooterSubsystem shooterSystem;
    private IntakeSubsystem intakeSystem;

    //Commands
    private Com_Drive driveCommand;
    private Com_Shooter shooterCommand;
    private Com_Intake intakeCommand;
    private Com_Outtake outtakeCommand;

    //Extranious
    private GamepadEx m_driverOp;
//    private Button slowDrive;
    private RevIMU imu;
    private FtcDashboard dashboard;
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

        //imu
        imu = new RevIMU(hardwareMap);
        imu.init();

        //Controller
        m_driverOp = new GamepadEx(gamepad1);
        dashboard = FtcDashboard.getInstance();

        //Subsystems and Commands
        driveSystem = new DriveSystem(fL, fR, bL, bR);
        driveCommand = new Com_Drive(driveSystem, m_driverOp::getLeftX, m_driverOp::getLeftY, m_driverOp::getRightX, ()->mult);

        shooterSystem = new ShooterSubsystem(flyWheel, telemetry);
        shooterCommand = new Com_Shooter(shooterSystem);

        intakeSystem = new IntakeSubsystem(intakeA);
        intakeCommand = new Com_Intake(intakeSystem);
        outtakeCommand = new Com_Outtake(intakeSystem);

//       Old Method no longer necessary:
//        slowDrive = new GamepadButton(m_driverOp, GamepadKeys.Button.Y)
//                .toggleWhenPressed(()->mult = 0.5, ()->mult = 1.0);

        m_driverOp.getGamepadButton(GamepadKeys.Button.Y)
                .toggleWhenPressed(()->mult = 0.5, ()->mult = 1.0);

        m_driverOp.getGamepadButton(GamepadKeys.Button.A).toggleWhenPressed(shooterCommand);
        
        m_driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).toggleWhenActive(intakeCommand);
        m_driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).toggleWhenActive(outtakeCommand);

        register(driveSystem);
        driveSystem.setDefaultCommand(driveCommand);
    }
}

