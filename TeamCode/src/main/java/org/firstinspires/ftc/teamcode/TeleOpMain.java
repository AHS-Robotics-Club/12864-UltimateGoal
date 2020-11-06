package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.command.CommandOpMode;

import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveSystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSystem;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_Drive;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_NoShoot;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_SetShootPower;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_Shoot;

@TeleOp(name = "CommandBaseTest")
public class TeleOpMain extends CommandOpMode {

    public int pwrSelect = 0;

    private Motor fL, bL, fR, bR;
    private MotorEx shot;

    private DriveSystem mecDrive;
    private Com_Drive driveCommand;
    private ShooterSystem shooterSystem;
    private Com_Shoot shootCommand;
    private Com_NoShoot stopCommand;
    private Com_SetShootPower shooterPowerCommand;

    public GamepadEx m_driverOp, m_toolOp;
    private Button shooterStart, shooterStop;
    private ButtonReader dPadUp, dPadDown;

    @Override
    public void initialize() {
        fL = new Motor(hardwareMap, "fL");
        fR = new Motor(hardwareMap, "fR");
        bL = new Motor(hardwareMap, "bL");
        bR = new Motor(hardwareMap, "bR");

        shot = new MotorEx(hardwareMap, "shot", Motor.GoBILDA.BARE);

        mecDrive = new DriveSystem(fL, fR, bL, bR);

        m_driverOp = new GamepadEx(gamepad1);
        m_toolOp = new GamepadEx(gamepad2);

        //this all is for shooter power adjustment:
        //the code below is for looping through modes 1-3
        //after a case statement within the ShooterSystem class checks the mode and sets the motor power
        dPadUp = new ButtonReader(m_toolOp, GamepadKeys.Button.DPAD_UP);
        dPadDown = new ButtonReader(m_toolOp, GamepadKeys.Button.DPAD_DOWN);

        if(dPadDown.wasJustPressed()){
            pwrSelect--;
            if(pwrSelect <= 0)
                pwrSelect = 3;
        }else if(dPadUp.wasJustPressed()){
            pwrSelect++;
            if(pwrSelect >= 4)
                pwrSelect = 1;
        }

        driveCommand = new Com_Drive(mecDrive, m_driverOp::getLeftX, m_driverOp::getLeftY, m_driverOp::getRightX);

        //IMPORTANT: Note to self remember in the Drive System class I just flipped the turn speed and strafe speed
        shooterSystem = new ShooterSystem(shot, this);
        shootCommand = new Com_Shoot(shooterSystem);
        stopCommand = new Com_NoShoot(shooterSystem);

        shooterStart = (new GamepadButton(m_driverOp, GamepadKeys.Button.A))
                .whenPressed(shootCommand);
        shooterStop = (new GamepadButton(m_driverOp, GamepadKeys.Button.B))
                .whenPressed(stopCommand);

        mecDrive.setDefaultCommand(driveCommand);

        register(mecDrive, shooterSystem);

        schedule(driveCommand, shooterPowerCommand);
    }
}


