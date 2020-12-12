package org.firstinspires.ftc.teamcode.jacksonSama.shhhnopeaking;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

//Make your pipline more beautiful UwU
public class BeautifulPipeline{
    private boolean rects;
    Mat matYCrCb = new Mat();
    Mat test = new Mat();
    private double topAverage;
    ArrayList<Mat> rectangle = new ArrayList<Mat>();
    ArrayList<Mat> block = new ArrayList<Mat>();
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
        rectangle.add(mat);
        Imgproc.rectangle(mat, rect, color, thickness);
    }
    public void drawRectMatrix(Mat mat, Rect rect, Scalar color){
        rectangle.add(mat);
        drawRectMatrix(mat, rect, color, 1);
    }
    public void drawRectMatrix(Rect rect, Scalar color){
        rectangle.add(new Mat());
        drawRectMatrix(rectangle.get(rectangle.size()-1), rect, color, 1);
    }

    public Rect createRect(double xPercentage, double yPercentage, int width, int height){
        Rect rectangle = new Rect(
                (int) (matYCrCb.width() * xPercentage),
                (int) (matYCrCb.height() * yPercentage),
                width, height);
        return rectangle;
    }
    public void submatAndExtract(Rect rect){
        block.add(matYCrCb.submat(rect));
//        Core.extractChannel(block.get(block.size()-1), new Mat(), 2);
        Core.extractChannel(block.get(block.size()-1), test, 2);
    }
    public void testAvg(){
        Scalar topMean = Core.mean(test);

        topAverage = topMean.val[0];

    }
    public double getAvg(){
        return topAverage;
    }
}
