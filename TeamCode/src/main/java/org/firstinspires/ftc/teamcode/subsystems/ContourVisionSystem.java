package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.jacksonSama.stolenVision.UGContourRingDetector;
import org.firstinspires.ftc.teamcode.jacksonSama.stolenVision.UGContourRingPipeline;

public class ContourVisionSystem extends SubsystemBase {
    private UGContourRingDetector ugContourRingDetector;
    private Telemetry telemetry;

    public ContourVisionSystem(UGContourRingDetector gaussianDetecty, Telemetry telemetryIn){
        ugContourRingDetector = gaussianDetecty;
        telemetry = telemetryIn;
    }
    public enum Size{
        ZERO, ONE, FOUR
    }
    public VisionSystem.Size getStackSize(){
        UGContourRingPipeline.Height stack = ugContourRingDetector.getHeight();
        switch (stack) {
            case ZERO:
                telemetry.addData("Rings Detected", "Zero");
                telemetry.update();
                return VisionSystem.Size.ZERO;
            case ONE:
                telemetry.addData("Rings Detected", "One");
                telemetry.update();
                return VisionSystem.Size.ONE;
            case FOUR:
                telemetry.addData("Rings Detected", "Four");
                telemetry.update();
                return VisionSystem.Size.FOUR;
            default:
                break;
        }
        return null;
    }

}
