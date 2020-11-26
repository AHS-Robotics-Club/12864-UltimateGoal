package org.firstinspires.ftc.teamcode.jacksonSama;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveSystem;
import org.firstinspires.ftc.teamcode.subsystems.commands.drive.Com_Drive;

@TeleOp(name="TeleDriveOnly")
public class TeleopDriveOnly extends CommandOpMode {


    private Motor fL, bL, fR, bR;

    private DriveSystem mecDrive;
    private Com_Drive driveCommand;
    private RevIMU imu;
    public GamepadEx m_driverOp, m_toolOp;

    @Override
    public void initialize() {
        fL = new Motor(hardwareMap, "fL");
        fR = new Motor(hardwareMap, "fR");
        bL = new Motor(hardwareMap, "bL");
        bR = new Motor(hardwareMap, "bR");

        imu = new RevIMU(hardwareMap);
        imu.init();
        //one of our motors is messed up so it has to be inverted woooooo
        bL.setInverted(true);

        mecDrive = new DriveSystem(fL, fR, bL, bR, imu);

        m_driverOp = new GamepadEx(gamepad1);
        m_toolOp = new GamepadEx(gamepad2);

        driveCommand = new Com_Drive(mecDrive, m_driverOp::getLeftX, m_driverOp::getLeftY, m_driverOp::getRightX);

        mecDrive.setDefaultCommand(driveCommand);

        register(mecDrive, new SubsystemBase(){
            @Override
            public void periodic() {
                telemetry.addData("imu heading", imu.getHeading());
                telemetry.update();
            }
        });

        schedule(driveCommand);
    }
}


