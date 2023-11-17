package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.Color_Detector;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@TeleOp(name = "AprilTagsColorDetectorToo", group = "Detector")


public class AprilTagsColorDetector extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    //The variable to store our instance of the AprilTag processor.
    private AprilTagProcessor aprilTag;
    //The variable to store our instance of the ColorDetector Object Detection processor.
    private Color_Detector colorDetector;
    //The variable to store our instance of the vision portal.
    private VisionPortal myVisionPortal;

    @Override
    public void runOpMode() {
        initDoubleVision();
        while (!isStopRequested()) {

            if (opModeInInit()) {
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
                if (myVisionPortal.getProcessorEnabled(colorDetector)) {
                    telemetry.addLine("Dpad Down to disable ColorDetector");
                    telemetry.addLine();
                    telemetryColorDetector();
                } else {
                    telemetry.addLine("Dpad Up to enable ColorDetector");
                }

            }
        }
    }
    //Initialize AprilTag and TFOD.
    private void initDoubleVision() {
        // -----------------------------------------------------------------------------------------
        // AprilTag Configuration
        // -----------------------------------------------------------------------------------------

        aprilTag = new AprilTagProcessor.Builder()
                .build();

        // -----------------------------------------------------------------------------------------
        // TFOD Configuration
        // -----------------------------------------------------------------------------------------

        colorDetector = new Color_Detector()
                .build();

        // -----------------------------------------------------------------------------------------
        // Camera Configuration
        // -----------------------------------------------------------------------------------------

        if (USE_WEBCAM) {
            myVisionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessors(colorDetector, aprilTag)
                    .build();
        } else {
            myVisionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessors(colorDetector, aprilTag)
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

}   // end method telemetryAprilTag()
    private void telemetryColorDetector() {
        List<Recognition> currentRecognitions = colorDetector.getLocation();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()

}   // end class



