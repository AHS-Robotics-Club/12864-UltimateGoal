package org.firstinspires.ftc.teamcode.drive.opmode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.vision.UGContourRingDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.Com_Intake;
import org.firstinspires.ftc.teamcode.commands.Com_Outtake;
import org.firstinspires.ftc.teamcode.commands.drive.Com_Drive;
import org.firstinspires.ftc.teamcode.commands.groups.FourRing;
import org.firstinspires.ftc.teamcode.commands.groups.OneRing;
import org.firstinspires.ftc.teamcode.commands.groups.ZeroRing;
import org.firstinspires.ftc.teamcode.subsystems.ContourVisionSystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSystem;

import java.util.HashMap;

@Autonomous(name="Aiden")
public class VisionTesting extends CommandOpMode {

    private UGContourRingDetector ugContourRingDetector;
    private ContourVisionSystem visionSystem;
    private VisionSystem.Size height;

    private IntakeSubsystem intakeSubsystem;

    @Override
    public void initialize() {
        ugContourRingDetector = new UGContourRingDetector(hardwareMap, "poopcam", telemetry, true);
        ugContourRingDetector.init();


        height = VisionSystem.Size.ZERO;
        while (!isStarted()) {
            height = visionSystem.getStackSize();
        }
//        schedule(new SelectCommand(new HashMap<Object, Command>() {{
//            put(VisionSystem.Size.ZERO, (new Com_Intake(intakeSubsystem)));
//            put(VisionSystem.Size.ONE, (new Com_Outtake(intakeSubsystem)));
//            put(VisionSystem.Size.FOUR, (new Com_Outtake(intakeSubsystem));
//        }},()->height))
//    });
    }
}
