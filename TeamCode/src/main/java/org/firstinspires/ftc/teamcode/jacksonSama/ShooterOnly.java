package org.firstinspires.ftc.teamcode.jacksonSama;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DriveSystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSystem;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_Drive;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_NoShoot;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_Shoot;

@TeleOp(name="shooteronly")
public class ShooterOnly extends CommandOpMode {

    public double pwrSelect = 1.0;

    private Motor fL, bL, fR, bR;
    private MotorEx shot;

    private DriveSystem mecDrive;
    private Com_Drive driveCommand;
    //Shooter subsystem and commands initialization
    private ShooterSystem shooterSystem;
    private Com_Shoot shootCommand;
    private Com_NoShoot stopCommand;

    public GamepadEx m_driverOp, m_toolOp;
    private Button toggleShooter, dpadUp, dpadDown;

    @Override
    public void initialize() {
        fL = new Motor(hardwareMap, "fL");
        fR = new Motor(hardwareMap, "fR");
        bL = new Motor(hardwareMap, "bL");
        bR = new Motor(hardwareMap, "bR");


        //one of our motors is messed up so it has to be inverted woooooo
        bL.setInverted(true);

        shot = new MotorEx(hardwareMap, "shot", Motor.GoBILDA.BARE);
        shot.setRunMode(Motor.RunMode.VelocityControl);

        mecDrive = new DriveSystem(fL, fR, bL, bR);

        m_driverOp = new GamepadEx(gamepad1);
        m_toolOp = new GamepadEx(gamepad2);

        dpadDown = new GamepadButton(m_driverOp, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> {
                    if (pwrSelect < 0.05) {
                        pwrSelect = 1;
                    } else {
                        pwrSelect -= 0.25;
                    }
                }));
        dpadUp = new GamepadButton(m_driverOp, GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> {
                    if (pwrSelect > 0.95) {
                        pwrSelect = 0;
                    } else {
                        pwrSelect += 0.25;
                    }
                }));

        driveCommand = new Com_Drive(mecDrive, m_driverOp::getRightX, m_driverOp::getLeftY, m_driverOp::getLeftX);

        shooterSystem = new ShooterSystem(shot, telemetry, () -> pwrSelect);
        shootCommand = new Com_Shoot(shooterSystem);
        stopCommand = new Com_NoShoot(shooterSystem);
        toggleShooter = new GamepadButton(m_driverOp, GamepadKeys.Button.A)
                .whenPressed(new ConditionalCommand(shootCommand, stopCommand, shooterSystem::active))
                .whenReleased(new InstantCommand(shooterSystem::toggle));

        mecDrive.setDefaultCommand(driveCommand);

        register(mecDrive, shooterSystem);

        schedule(driveCommand);
    }
}
