package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "RedClose")
public class RedClose extends LinearOpMode {

    DriveTrain driveTrain;
    ColorDetector colorDetector = new ColorDetector(telemetry, true);
    Drawing drawing;
    Camera camera;
    ColorDetector.Location location;
    Lift lift;
    OutTake outTake;
    ElapsedTime timer = new ElapsedTime();
    Arm arm;
    private VisionPortal visionPortal;


    @Override
    public void runOpMode() {
        DriveTrain driveTrain = new DriveTrain(hardwareMap, this);
        Drawing drawing = new Drawing(hardwareMap, this);
        driveTrain.setStartPos(0, 0, 0);
        Arm arm = new Arm(hardwareMap, this);
        Lift lift = new Lift(hardwareMap, this);

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(800, 448));
        builder.enableLiveView(true);
        builder.addProcessor(colorDetector);
        visionPortal = builder.build();


        driveTrain.reset();
        while (opModeInInit()) {
            location = colorDetector.getLocation();

            sleep(50);
        }
        visionPortal.setProcessorEnabled(colorDetector, false);
        if (opModeIsActive() && !isStopRequested()) {
            if (location == ColorDetector.Location.MIDDLE) {


                driveTrain.driveTo(710, -110, 0, 3000);
                drawing.outtakeoTO();
                sleep(1500);
                drawing.stop();
                driveTrain.driveTo(40, -90, 0, 2000);
                driveTrain.driveTo(40, -90, 90, 2000);
                driveTrain.driveTo(40, 2000, 90, 2800);
                driveTrain.driveTo(40, 2100, 90, 1500);
                driveTrain.driveTo(250, 2100, 90, 1500);
                driveTrain.driveTo(250, 2100, 0, 2000);

            } else if (location == ColorDetector.Location.LEFT) {
                driveTrain.driveTo(300, -300, 0, 1500);
                driveTrain.driveTo(500, -300, 0, 1500);
                drawing.outtakeo3();
                sleep(1000);
                driveTrain.driveTo(100, -260, 0, 1500);
                drawing.stop();
                driveTrain.driveTo(40, -90, 0, 2000);
                driveTrain.driveTo(40, -90, 90, 2000);
                driveTrain.driveTo(40, 2000, 90, 2800);
                driveTrain.driveTo(40, 2100, 90, 1500);


                driveTrain.driveTo(250, 2100, 90, 1500);
                driveTrain.driveTo(250, 2100, 0, 2000);


            } else {
                driveTrain.driveTo(540, 0, 0, 2000);
                driveTrain.driveTo(540, 0, 45, 2000);
                driveTrain.driveTo(540, 200, 45, 2000);
                drawing.outtakeo3();
                driveTrain.driveTo(540, 0, 45, 2000);
                driveTrain.driveTo(40, -90, 0, 2000);
                drawing.stop();
                driveTrain.driveTo(40, -90, 90, 2000);
                driveTrain.driveTo(40, 2000, 90, 2800);
                driveTrain.driveTo(40, 2100, 90, 1500);
                driveTrain.driveTo(250, 2100, 90, 1500);
                driveTrain.driveTo(250, 2100, 0, 2000);
            }
        }
    }
}