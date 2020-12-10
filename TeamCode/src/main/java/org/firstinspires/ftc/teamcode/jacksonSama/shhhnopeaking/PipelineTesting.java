package org.firstinspires.ftc.teamcode.jacksonSama.shhhnopeaking;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvPipeline;

public class PipelineTesting extends OpenCvPipeline {
    BeautifulPipeline pipeline = new BeautifulPipeline(true);
    @Override
    public Mat processFrame(Mat input) {
        pipeline.RGBtoYCrCb(input);
        Rect rect1 = pipeline.createRect(0.25, 0.25, 25, 25);
        pipeline.drawRectMatrix(rect1, new Scalar(255, 0, 0));
        pipeline.submatAndExtract(rect1);
        return input;
    }
}
