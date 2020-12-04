package org.firstinspires.ftc.teamcode.jacksonSama.shhhnopeaking;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class BeautifulPipeline{
    private boolean rects;
    Mat matYCrCb = new Mat();

    public BeautifulPipeline(boolean usesRect){
        rects = usesRect;
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
    public void drawRectMatrix(Mat mat, Rect rect, Scalar color){
        drawRectMatrix(mat, rect, color, 1);
    }
    public void drawRectMatrix(Rect rect, Scalar color){
        Mat rectangle = new Mat();
        drawRectMatrix(rectangle, rect, color, 1);
    }

    public Rect createRect(double xPercentage, double yPercentage, int width, int height){
        Rect rectangle = new Rect(
                (int) (matYCrCb.width() * xPercentage),
                (int) (matYCrCb.height() * yPercentage),
                width, height);
        return rectangle;
    }
    public void submat(Rect rect){
        Mat block;
        block = matYCrCb.submat(rect);
        Core.extractChannel(block, block, 2);
        //Dont judge I know this looks dumb I just put a second block in there so no errors
        // WIP WIP WIP JACKSON GAYYYYY
    }
}
