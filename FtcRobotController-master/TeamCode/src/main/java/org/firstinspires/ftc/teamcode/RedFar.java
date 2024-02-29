package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ColorDetector.Location;
import org.firstinspires.ftc.vision.VisionPortal;



@Autonomous(name = "RedFar")
public class RedFar extends LinearOpMode {

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
                driveTrain.driveTo(650, 70, 0, 1500);
                drawing.outtakeoTO();
                sleep(1000);
                drawing.stop();
                driveTrain.driveTo(680, 780, 90, 2000);
                // outTake.PutOut1();
                driveTrain.driveTo(680, 700, 90, 2000);
                driveTrain.driveTo(680, 800, 90, 2000);



            }
            else if (location == ColorDetector.Location.RIGHT) {
                driveTrain.driveTo(300, 270, 0, 1500);
                driveTrain.driveTo(500, 270, 0, 1500);
                driveTrain.driveTo(100, 270, 0, 1500);
                driveTrain.driveTo(680, 780, 90, 2000);
                // outTake.PutOut1();
                driveTrain.driveTo(680, 700, 90, 2000);
                driveTrain.driveTo(680, 800, 90, 2000);


            }
            else {
                driveTrain.driveTo(530, 0, 0, 1000);
                driveTrain.driveTo(530, 0, -45, 1000);
                driveTrain.driveTo(530, -240, -45, 1000);
                driveTrain.driveTo(50, 50, 0, 1000);
                driveTrain.driveTo(680, 780, 90, 2000);
                // outTake.PutOut1();
                driveTrain.driveTo(680, 700, 90, 2000);
                driveTrain.driveTo(680, 800, 90, 2000);


            }

        }


    }
}