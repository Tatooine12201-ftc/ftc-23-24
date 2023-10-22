package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;


public class Color_Detector extends OpenCvPipeline{
    Telemetry telemetry;
    Mat mat = new Mat();


    static final Rect Left_Roi = new Rect(
            new Point(60, 35),
            new Point(120, 75));
    static final Rect Right_Roi = new Rect(
            new Point(140, 35),
            new Point(200, 75));
        public void TPD(Telemetry t) { telemetry = t;}




    List<MatOfPoint> contours;
    Mat hierachy;


    public Color_Detector(Telemetry t) { telemetry=t;}

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        // working blue
        Scalar lowHSV = new Scalar(110, 50, 50);
        Scalar hghHSV = new Scalar(130, 255, 255);

        /*
        // working red
        Scalar lowHSV = new Scalar(0, 50, 70);
        Scalar hghHSV = new Scalar(10, 255, 255);
         */
        Core.inRange(mat, lowHSV, hghHSV, mat);
        Mat left = mat.submat(Left_Roi);
        Mat right = mat.submat(Right_Roi);

        double leftValue = Core.sumElems(left).val[0] / Left_Roi.area() / 255;
        double rightValue = Core.sumElems(left).val[0] / Right_Roi.area() / 255;

        left.release();
        right.release();

        telemetry.addData("Left raw value", (int)Core.sumElems(left).val[0]);
        telemetry.addData("Right raw value", (int)Core.sumElems(right).val[0]);
        telemetry.addData("Left percentage",Math.round(leftValue*100)+"%");
        telemetry.addData("Right percentage",Math.round(rightValue*100)+"%");
        return mat;
    }


}
