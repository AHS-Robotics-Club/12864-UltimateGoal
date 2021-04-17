package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.commands.Com_Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.TimedAction;

@Config
@Autonomous
public class SingleMotorPID extends CommandOpMode {

    public static double VELOCITY = 2000;

    enum Mode {
        DRIVER_MODE,
        TUNING_MODE
    }

    public static double kP = 22, kI = 0, kD = 0.3, kS = 0, kV = 1.5;

    private GamepadEx gamepad;
    private Motor motor;
    private SimpleServo flicker;
    private ShooterSubsystem shooterSystem;
    private Com_Shooter shooterCommand;
    private Button xButton, aButton;
    private TimedAction flickerAction;
    private Mode mode;
    private double lastKp, lastKi, lastKd, lastKs, lastKv;
    private VoltageSensor voltageSensor;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor = new  Motor(hardwareMap, "shoot");
        motor.setRunMode(Motor.RunMode.VelocityControl);
        motor.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        flicker = new SimpleServo(hardwareMap, "flicker", 0, 270);
        gamepad = new GamepadEx(gamepad1);

        mode = Mode.TUNING_MODE;

        lastKp = kP;
        lastKi = kI;
        lastKd = kD;
        lastKs = kS;
        lastKv = kV;


        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

        flickerAction = new TimedAction(
                ()-> flicker.setPosition(0.37),
                ()-> flicker.setPosition(0.6),
                115,
                true
        );

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        shooterSystem = new ShooterSubsystem(motor, flicker, flickerAction, voltageSensor);
        shooterCommand = new Com_Shooter(shooterSystem);

        schedule(new RunCommand(() -> telemetry.addData("mode", mode)));

        xButton = new GamepadButton(gamepad, GamepadKeys.Button.X)
                .whenPressed(() -> {
                    mode = Mode.DRIVER_MODE;
                    motor.setRunMode(Motor.RunMode.RawPower);
                });
        aButton = new GamepadButton(gamepad, GamepadKeys.Button.A)
                .whenPressed(() -> {
                    motor.setRunMode(Motor.RunMode.VelocityControl);
                    mode = Mode.TUNING_MODE;
                });

        gamepad.getGamepadButton(GamepadKeys.Button.Y).whenHeld(shooterCommand);


        schedule(new RunCommand(() -> {
            switch (mode) {
                case TUNING_MODE:
                    double velocity = motor.getCorrectedVelocity();
                    motor.set(VELOCITY/motor.ACHIEVABLE_MAX_TICKS_PER_SECOND);
                    // update telemetry
                    telemetry.addData("targetVelocity", VELOCITY);
                    telemetry.addData("measuredVelocity", velocity);
                    telemetry.addData(
                            "error",
                            VELOCITY - velocity
                    );
                    break;
                case DRIVER_MODE:
                    motor.set(gamepad1.left_stick_y);
                    break;
            }

            if (lastKp != kP || lastKi != kI || lastKd != kD
                    || lastKs != kS || lastKv != kV) {
                motor.setVeloCoefficients(kP, kI, kD);
                motor.setFeedforwardCoefficients(kS, kV);

                lastKp = kP;
                lastKi = kI;
                lastKd = kD;
                lastKs = kS;
                lastKv = kV;
            }

            telemetry.update();
        }));
    }

}
