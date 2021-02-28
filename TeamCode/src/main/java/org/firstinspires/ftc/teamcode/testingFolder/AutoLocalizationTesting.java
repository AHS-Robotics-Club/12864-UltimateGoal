package org.firstinspires.ftc.teamcode.testingFolder;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.rr.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

@TeleOp
public class AutoLocalizationTesting extends CommandOpMode {

    private MecanumDriveSubsystem drive;
    private MecanumDriveCommand driveCommand;
    private GamepadEx gamepad;

    @Override
    public void initialize() {
        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), false);

        drive.setPoseEstimate(new Pose2d(-63.0, -40.0, Math.toRadians(180.0)));

        gamepad = new GamepadEx(gamepad1);

        schedule(new RunCommand(() -> {
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }));

        driveCommand = new MecanumDriveCommand(
                drive, () -> -gamepad.getLeftY(),
                gamepad::getLeftX, gamepad::getRightX
        );

        schedule(driveCommand);
    }

}
