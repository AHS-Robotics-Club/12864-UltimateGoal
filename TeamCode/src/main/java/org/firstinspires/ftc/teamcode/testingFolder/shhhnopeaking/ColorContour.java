package org.firstinspires.ftc.teamcode.testingFolder.shhhnopeaking;


import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class ColorContour {

    Scalar upper, lower;

    public ColorContour(Scalar lowerBound, Scalar upperBound){
        lower = lowerBound;
        upper = upperBound;
    }

    public Mat findAndDrawContours(Mat image, Mat frame){
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        Imgproc.findContours(image, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        if(hierarchy.size().height > 0 && hierarchy.size().width > 0){
            for(int index = 0; index >= 0; index = (int) hierarchy.get(0, index)[0])
                Imgproc.drawContours(frame, contours, index, new Scalar(250, 0, 0));
        }
        return frame;
    }

}
