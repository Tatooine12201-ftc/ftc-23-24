package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.vision.VisionPortal;
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

@TeleOp(name = "AprilTagsColorDetectorToo", group = "Detector")
public class Color_Detector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    //The variable to store our instance of the AprilTag processor.
    private AprilTagProcessor aprilTag;
    //The variable to store our instance of the ColorDetector Object Detection processor.
    private Color_Detector colorDetector;
    //The variable to store our instance of the vision portal.
    private VisionPortal myVisionPortal;

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

    List<MatOfPoint> contours;
    Mat hierachy;


    public Color_Detector(Telemetry t) {
        telemetry = t;
    }

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
    public void runOpMode()throws InterruptedException {
       // initDoubleVision();
        while (!isStopRequested() ) {

            if (IsActive()) {
                telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
                telemetry.addLine();
                telemetry.addLine("----------------------------------------");
                if (myVisionPortal.getProcessorEnabled(aprilTag)) {
                    // User instructions: Dpad left or Dpad right.
                    telemetry.addLine("Dpad Left to disable AprilTag");
                    telemetry.addLine();
                    telemetryAprilTag();
                } else {
                    telemetry.addLine("Dpad Right to enable AprilTag");
                }
                telemetry.addLine();
                telemetry.addLine("----------------------------------------");
                ///  if (myVisionPortal.getProcessorEnabled(colorDetector)) {
                telemetry.addLine("Dpad Down to disable ColorDetector");
                telemetry.addLine();

            } else {
                telemetry.addLine("Dpad Up to enable ColorDetector");
            }

        }
    }

    private boolean isStopRequested() {
        return false;
    }

    private boolean IsActive (){
        return true;
    }

    // }
    //Initialize AprilTag and TFOD.
    private void initDoubleVision(HardwareMap hw) {
        // -----------------------------------------------------------------------------------------
        // AprilTag Configuration
        // -----------------------------------------------------------------------------------------
        aprilTag = new AprilTagProcessor.Builder()
                .build();

        // -----------------------------------------------------------------------------------------
        // Camera Configuration
        // -----------------------------------------------------------------------------------------

        if (USE_WEBCAM) {

            myVisionPortal = new VisionPortal.Builder()
                    .setCamera(hw.get(WebcamName.class, "Webcam 1"))
                    //   .addProcessors(colorDetector, aprilTag)
                    .build();
        } else {
            myVisionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    // .addProcessors(colorDetector, aprilTag)
                    .build();
        }
    }   // end initDoubleVision()
    //Add telemetry about AprilTag detections.
    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

    }
    // end method telemetryAprilTag()



}

