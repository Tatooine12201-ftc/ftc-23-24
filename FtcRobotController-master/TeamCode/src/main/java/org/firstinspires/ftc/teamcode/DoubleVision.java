package org.firstinspires.ftc.teamcode;
import android.location.Location;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.teamcode.Color_Detector;

import java.util.List;
@TeleOp(name = " Double Vision", group = " Vision")
 public class DoubleVision  extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    //private AprilTagProcessor aprilTag;
    private AprilTags.ConceptAprilTag aprilTag;
    private VisionPortal myVisionPortal;
    private Color_Detector.Location location;
    private  boolean GetPipline =true ;


    @Override
    public void runOpMode() {
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
        if (GetPipline) {
            telemetry.addData("apriltag", aprilTag);
            GetPipline = false;
        }
        else {
            telemetry.addData("location",location);
        }
        telemetry.update();

        }


    }
}
