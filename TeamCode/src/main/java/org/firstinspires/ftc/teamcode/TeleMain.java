package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.testingFolder.shhhnopeaking.GamepadButtonB;
import org.firstinspires.ftc.teamcode.subsystems.DriveSystem;
import org.firstinspires.ftc.teamcode.subsystems.commands.drive.Com_Drive;

@TeleOp(name="KekW")
public class TeleMain extends CommandOpMode {
    //Servos and Motors
    private Motor fL, fR, bL, bR;

    //Subsystems
    private DriveSystem driveSystem;

    //Commands
    private Com_Drive driveCommand;

    //Extranious
    private GamepadEx m_driverOp;
    private Button slowDrive;
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

        bL.setInverted(true);

        //imu
        imu = new RevIMU(hardwareMap);
        imu.init();

        //Controller
        m_driverOp = new GamepadEx(gamepad1);
        dashboard = FtcDashboard.getInstance();

        //Subsystems and Commands
        driveSystem = new DriveSystem(fL, fR, bL, bR);
        driveCommand = new Com_Drive(driveSystem, m_driverOp::getLeftX, m_driverOp::getLeftY, m_driverOp::getRightX, ()->mult);
        slowDrive = new GamepadButtonB(m_driverOp, GamepadKeys.Button.Y)
                .toggleWhenPressed(()->mult = 0.5, ()->mult = 1.0);

        register(driveSystem);
        driveSystem.setDefaultCommand(driveCommand);
    }
}

