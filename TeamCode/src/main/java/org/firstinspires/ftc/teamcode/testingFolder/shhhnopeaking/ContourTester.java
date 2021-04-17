package org.firstinspires.ftc.teamcode.testingFolder.shhhnopeaking;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ContourTester extends OpenCvPipeline {

    Scalar low = new Scalar(0.0, 141.0, 0.0);
    Scalar high = new Scalar(255.0, 230.0, 95.0);

    ColorContour colorContour = new ColorContour(low, high);
    Mat mask;
    Mat frame;
    Mat output;

    @Override
    public Mat processFrame(Mat input) {
        frame = new Mat();
        output = new Mat();
        mask = new Mat(output.rows(), output.cols(), CvType.CV_8UC1);

        Imgproc.cvtColor(input, output, Imgproc.COLOR_RGB2YCrCb);

        Core.inRange(output, low, high, mask);
        Core.bitwise_and(input, input, frame, mask);

        Imgproc.GaussianBlur(mask, mask, new Size(7, 10), 0.0);

        return colorContour.findAndDrawContours(mask, frame);
    }
}
