package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.jacksonSama.shhhnopeaking.BetterToggle;
import org.firstinspires.ftc.teamcode.jacksonSama.shhhnopeaking.GamepadButtonB;
import org.firstinspires.ftc.teamcode.subsystems.DriveSystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSystem;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSystem;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_IntakeStart;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_NoShoot;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_OuttakeStart;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_PickUp;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_PutDown;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_Shoot;
import org.firstinspires.ftc.teamcode.subsystems.commands.drive.Com_Drive;

@TeleOp(name="Kanye South")
public class TeleopComp2 extends CommandOpMode {

    public double pwrSelect = 1.0;

    private Motor fL, bL, fR, bR;
    private Motor shot, intake, wobble;
    private CRServo servo;

    private DriveSystem mecDrive;
    private Com_Drive driveCommand;
    //Shooter subsystem and commands initialization
    private ShooterSystem shooterSystem;
    private Com_Shoot shootCommand;
    private Com_NoShoot stopCommand;

    private IntakeSystem intakeSystem;
    private Com_IntakeStart intakeStartCommand;
    private Com_OuttakeStart outtakeStartCommand;

    private WobbleSystem wobbleSystem;
    private Com_PickUp pickUpCommand;
    private Com_PutDown putDownCommand;

    private GamepadEx m_driverOp, m_toolOp;
    private Button toggleShooter, dpadUp, dpadDown, intakeOn, outtakeOn, wobbleButton, wobbleTwo;
    private Trigger leftTrigger, rightTrigger;
    private TriggerReader leftTriggerReader, rightTriggerReader;
    private BetterToggle wobbleToggle;
    private RevIMU imu;
    private ElapsedTime elapsedTime;
    private FunctionalCommand openCommand, closeCommand;

    @Override
    public void initialize() {
        fL = new Motor(hardwareMap, "fL");
        fR = new Motor(hardwareMap, "fR");
        bL = new Motor(hardwareMap, "bL");
        bR = new Motor(hardwareMap, "bR");
        imu = new RevIMU(hardwareMap);


        //one of our motors is messed up so it has to be inverted woooooo
        bL.setInverted(true);
        intake = new Motor(hardwareMap, "intake", Motor.GoBILDA.BARE);
        shot = new Motor(hardwareMap, "shot", Motor.GoBILDA.BARE);
        wobble = new Motor(hardwareMap, "wobble");
        wobble.setRunMode(Motor.RunMode.PositionControl);
        wobble.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        servo = new CRServo(hardwareMap, "servo");
//        shot.setRunMode(Motor.RunMode.VelocityControl);

        mecDrive = new DriveSystem(fL, fR, bL, bR);

        m_driverOp = new GamepadEx(gamepad1);
        m_toolOp = new GamepadEx(gamepad2);
        elapsedTime = new ElapsedTime();


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

        driveCommand = new Com_Drive(mecDrive, m_driverOp::getLeftX, m_driverOp::getLeftY, m_driverOp::getRightX);

        shooterSystem = new ShooterSystem(shot, telemetry, () -> pwrSelect);
        shootCommand = new Com_Shoot(shooterSystem);
        stopCommand = new Com_NoShoot(shooterSystem);
        toggleShooter = new GamepadButton(m_driverOp, GamepadKeys.Button.A)
                .toggleWhenPressed(shootCommand);
        intakeSystem = new IntakeSystem(intake);
        intakeStartCommand = new Com_IntakeStart(intakeSystem);
        outtakeStartCommand = new Com_OuttakeStart(intakeSystem);
       intakeOn = new GamepadButton(m_driverOp, GamepadKeys.Button.RIGHT_BUMPER)
                .whenHeld(intakeStartCommand);
       outtakeOn = new GamepadButton(m_driverOp, GamepadKeys.Button.LEFT_BUMPER)
               .whenHeld(outtakeStartCommand);

        wobbleSystem = new WobbleSystem(servo, wobble, telemetry, this::isStopRequested);
        pickUpCommand = new Com_PickUp(wobbleSystem);
        putDownCommand = new Com_PutDown(wobbleSystem);
        wobbleButton = new GamepadButton(m_driverOp, GamepadKeys.Button.X)
                .whenPressed(pickUpCommand);
        wobbleButton = new GamepadButton(m_driverOp, GamepadKeys.Button.Y)
                .whenPressed(putDownCommand);

        leftTriggerReader = new TriggerReader(m_driverOp, GamepadKeys.Trigger.LEFT_TRIGGER);
        rightTriggerReader = new TriggerReader(m_driverOp, GamepadKeys.Trigger.RIGHT_TRIGGER);
         openCommand = new FunctionalCommand(
                 () -> { return; }, wobbleSystem::putMeDownUwU,
                bool -> wobbleSystem.servoStop(), () -> false, wobbleSystem);

         closeCommand = new FunctionalCommand(
                 () -> { return; }, wobbleSystem::spinMeRightRoundBaby,
                bool -> wobbleSystem.servoStop(), () -> false, wobbleSystem);

         leftTrigger = new Trigger(() -> {
            leftTriggerReader.readValue();
            return leftTriggerReader.isDown();
        }).whileActiveContinuous(openCommand);

         rightTrigger = new Trigger(() -> {
            rightTriggerReader.readValue();
            return rightTriggerReader.isDown();
        }).whileActiveContinuous(closeCommand);

        mecDrive.setDefaultCommand(driveCommand);

        register(mecDrive);

        schedule(driveCommand);
    }
}
