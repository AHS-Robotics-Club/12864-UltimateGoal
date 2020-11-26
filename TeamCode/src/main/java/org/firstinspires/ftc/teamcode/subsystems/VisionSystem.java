package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.vision.UGRectDetector;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class VisionSystem extends SubsystemBase {
    private UGRectDetector ugRectDetector;
    private Telemetry telemetry;

    public enum Size{
        ZERO, ONE, FOUR
    }

    public VisionSystem(UGRectDetector rectyDetecty, Telemetry telemetryIn){
        ugRectDetector = rectyDetecty;
        telemetry = telemetryIn;
    }

    public Size getStackSize(){
        UGRectDetector.Stack stack = ugRectDetector.getStack();
        switch (stack) {
            case ZERO:
                telemetry.addData("Rings Detected", "Zero");
                telemetry.update();
                return Size.ZERO;
            case ONE:
                telemetry.addData("Rings Detected", "One");
                telemetry.update();
                return Size.ONE;
            case FOUR:
                telemetry.addData("Rings Detected", "Four");
                telemetry.update();
                return Size.FOUR;
            default:
                break;
        }
        return null;
    }
}
