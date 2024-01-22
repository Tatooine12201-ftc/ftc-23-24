
        package org.firstinspires.ftc.teamcode;

        import org.firstinspires.ftc.vision.VisionPortal;
        import org.firstinspires.ftc.vision.VisionProcessor;
        import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
        import org.opencv.core.Core;
        import org.opencv.core.MatOfPoint;
        import org.opencv.core.Point;
        import org.opencv.core.Rect;
        import org.opencv.core.Scalar;
        import org.opencv.imgproc.Imgproc;
        import org.openftc.easyopencv.OpenCvPipeline;
        import org.firstinspires.ftc.robotcore.external.Telemetry;
        import org.opencv.core.Mat;
        import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
        import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
        import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
        import java.util.List;

public abstract class ColorDetector implements VisionProcessor {
    Telemetry telemetry;
    Mat mat = new Mat();
    private static final boolean USE_WEBCAM = true;  // true for webcam, fal

    // se for phone camera
    //The variable to store our instance of the AprilTag processor.

    //The variable to store our instance of the ColorDetector Object Detection processor.

    //The variable to store our instance of the vision portal.
    // ??
    private VisionPortal myVisionPortal;

   // public static ColorDetector easyCreateWithDefaults()
 //   {
  //      return new ColorDetector.Builder().build();
  //  }
   // public static class Builder{

   // }


    public enum Location {
        LEFT,
        MIDDLE,
        RIGHT,
        NOT_FOUND
    }

    private Location location;

    static final Rect LEFT_ROI = new Rect(
            new Point(40, 40),
            new Point(253, 408));
    static final Rect MIDDLE_ROI = new Rect(
            new Point(292, 40),
            new Point(506, 408));
    static final Rect RIGHT_ROI = new Rect(
            new Point(546, 40),
            new Point(759, 408));
    static double COLOR_THRESHOLD = 0.4;

   // List<ColorDetector> contours;
    Mat hierachy;


    public ColorDetector(Telemetry t) {
        telemetry = t;
    }

   // @Override
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
        Mat left = mat.submat(LEFT_ROI);
        Mat right = mat.submat(RIGHT_ROI);
        Mat middle = mat.submat(MIDDLE_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;
        double middleValue = Core.sumElems(middle).val[0] / MIDDLE_ROI.area() / 255;

        left.release();
        right.release();
        middle.release();

        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");
        telemetry.addData("middle raw value", (int) Core.sumElems(middle).val[0]);
        telemetry.addData("middle percentage", Math.round(middleValue * 100) + "%");


        boolean Left = leftValue > rightValue && leftValue > middleValue;
        boolean Right = rightValue > leftValue && rightValue > middleValue;
        boolean Middle = middleValue > rightValue && middleValue > leftValue;

        if (Middle) {
            location = Location.MIDDLE;
            telemetry.addData("TeamProp Location", "middle");
        } else if (Right) {
            location = Location.RIGHT;
            telemetry.addData("TeamProp Location", "right");
        } else if (Left) {
            location = Location.LEFT;
            telemetry.addData("TeamProp Location", "left");
        } else {
            location = Location.NOT_FOUND;
            telemetry.addData("TeamProp Location", "not found");
        }
        telemetry.update();



        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar Acolor_cub = new Scalar(0, 255, 0);
        Scalar Bcolor_cub = new Scalar(255, 0, 0);

        Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT ? Acolor_cub : Bcolor_cub);
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT ? Acolor_cub : Bcolor_cub);
        Imgproc.rectangle(mat, MIDDLE_ROI, location == Location.MIDDLE ? Acolor_cub : Bcolor_cub);
        return mat;
    }


    public Location getLocation() {
        return location;


    }



}


