package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "BlueClose")
public class BlueClose extends LinearOpMode {

    DriveTrain driveTrain;
    ColorDetector colorDetector = new ColorDetector(telemetry, false);
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
                driveTrain.driveTo(650, 70, 0, 3000);
                drawing.outtakeoTO();
                sleep(3000);
                driveTrain.driveTo(650, 0, 0, 3000);
                driveTrain.driveTo(300, -50, 0, 3000);
                drawing.stop();
                sleep(1000);
                driveTrain.driveTo(0,0,0,3000);
                driveTrain.driveTo(100, 0, 0, 2000);
                driveTrain.driveTo(100, 0, -90, 2000);
                driveTrain.driveTo(100, -2400, -90, 2000);



            }
            if (location == ColorDetector.Location.RIGHT) {
                driveTrain.driveTo(0, 270, 0, 2000);
                driveTrain.driveTo(500, 270, 0, 2000);
                driveTrain.driveTo(100, 270, 0, 2000);
                driveTrain.driveTo(100, 270, -90, 2000);
                driveTrain.driveTo(100, -2400, -90, 2000);


            }
            else {
                driveTrain.driveTo(580, 0, 0, 1000);
                driveTrain.driveTo(580, 0, -90, 1000);
                driveTrain.driveTo(580, -120, -90, 1000);
                driveTrain.driveTo(580, 0, -90, 1000);
                driveTrain.driveTo(0,0,0,3000);
                driveTrain.driveTo(100, 0, 0, 2000);
                driveTrain.driveTo(100, 0, -90, 2000);
                driveTrain.driveTo(100, -2400, -90, 2000);


            }

        }


    }
}