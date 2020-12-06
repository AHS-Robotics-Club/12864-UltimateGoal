package org.firstinspires.ftc.teamcode.jacksonSama.shhhnopeaking;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

//Make your pipline more beautiful UwU
public class BeautifulPipeline{
    private boolean rects;
    Mat matYCrCb = new Mat();
    Mat rectangle;
    public BeautifulPipeline(boolean usesRect){
        rects = usesRect;
    }
    /**
     * Convert the input frame to the color space matYCrCb
     * Then stores this converted color space in the mat matYCrCb
     * @param input takes the input from the process frame
     */
    public void RGBtoYCrCb(Mat input){
        Imgproc.cvtColor(input, matYCrCb, Imgproc.COLOR_RGB2YCrCb);
    }
    /**
     * Draw the rectangle onto the desired mat
     * @param mat   The mat that the rectangle should be drawn on
     * @param rect  The rectangle
     * @param color The color the rectangle will be
     * @param thickness The thickness of the rectangle
     */
    public void drawRectMatrix(Mat mat, Rect rect, Scalar color, int thickness){
        rectangle = mat;
        Imgproc.rectangle(mat, rect, color, thickness);
    }
    public void drawRectMatrix(Mat mat, Rect rect, Scalar color){
        drawRectMatrix(mat, rect, color, 1);
    }
    public void drawRectMatrix(Rect rect, Scalar color){
        rectangle = new Mat();
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

    }
}
