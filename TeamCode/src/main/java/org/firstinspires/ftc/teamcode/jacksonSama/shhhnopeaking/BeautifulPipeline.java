package org.firstinspires.ftc.teamcode.jacksonSama.shhhnopeaking;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class BeautifulPipeline{
    private boolean rects;
    public BeautifulPipeline(boolean usesRect){
        rects = usesRect;
    }
    public void drawRectMatrix(Mat mat, Rect rect, Scalar color){
        Imgproc.rectangle(mat, rect, color, 1);
    }
    /**
     * Draw the rectangle onto the desired mat
     * @param mat   The mat that the rectangle should be drawn on
     * @param rect  The rectangle
     * @param color The color the rectangle will be
     * @param thickness The thickness of the rectangle
     */
    public void drawRectMatrix(Mat mat, Rect rect, Scalar color, int thickness){
        Imgproc.rectangle(mat, rect, color, thickness);
    }

}
