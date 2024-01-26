package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.AprilTags;
import org.openftc.easyopencv.OpenCvPipeline;
//import org.firstinspires.ftc.teamcode.Basic.AprilTagCamera.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.AprilTags.ConceptAprilTag;

    @Autonomous(name = "Autonomous", group = "OpMode")
public class Camera extends LinearOpMode {
    OpenCvCamera camera;

        private boolean isRuning() {
        return opModeIsActive() && !isStopRequested();
    }

    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        //ColorDetector detector = new ColorDetector (Telemetry);
       // camera.setPipeline(detector);


        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }

        });

        waitForStart();


    }


}



