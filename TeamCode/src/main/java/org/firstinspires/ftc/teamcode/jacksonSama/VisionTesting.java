package org.firstinspires.ftc.teamcode.jacksonSama;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.vision.UGRectDetector;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="poopee")
public class VisionTesting extends CommandOpMode {
    UGRectDetector UGRectDetector;
    Motor motor;
    @Override
    public void initialize() {
        UGRectDetector = new UGRectDetector(hardwareMap);
        UGRectDetector.init();
    }

    @Override
    public void run() {
        UGRectDetector.Stack stack = UGRectDetector.getStack();
        motor = new Motor(hardwareMap, "test", Motor.GoBILDA.BARE);

        switch (stack) {
            case ZERO:
                telemetry.addData("Zero", "Weve got 0");
                telemetry.update();
                motor.set(0);
                break;
            case ONE:
                telemetry.addData("One", "Almost");
                telemetry.update();
                motor.set(0);
                break;
            case FOUR:
                telemetry.addData("Four", "FULL MATCH");
                telemetry.speak("Kono Dio Da!");
                telemetry.update();
                motor.set(1);

                break;
            default:
                break;
        }
    }
}