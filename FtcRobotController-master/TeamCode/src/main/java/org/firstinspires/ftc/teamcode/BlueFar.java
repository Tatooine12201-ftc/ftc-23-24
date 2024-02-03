package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ColorDetector.Location;
import org.firstinspires.ftc.vision.VisionPortal;



@Autonomous(name = "BlueFar")
public class BlueFar extends LinearOpMode{

    DriveTrain driveTrain;
    ColorDetector colorDetector = new ColorDetector(telemetry, false);
    Drawing drawing;
    Camera camera;
    Location location;

    ElapsedTime timer = new ElapsedTime();
    private VisionPortal visionPortal;


    @Override
    public void runOpMode() {
        DriveTrain driveTrain = new DriveTrain(hardwareMap, this);
        Drawing drawing = new Drawing(hardwareMap,this);
        driveTrain.setStartPos(0, 0, 0);

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(800,448));
        builder.enableLiveView(true);
        builder.addProcessor(colorDetector);
        visionPortal = builder.build();



        driveTrain.reset();
        while (opModeInInit()){
            location = colorDetector.getLocation();

            sleep(50);
        }
        visionPortal.setProcessorEnabled(colorDetector,false);
        visionPortal.stopStreaming();
        if (opModeIsActive() && !isStopRequested()) {
            if (location == Location.MIDDLE) {

            driveTrain.driveTo(702, 0, 0, 200);

        }
          else if (location == Location.LEFT) {
              driveTrain.driveTo(702, 0,-90,200);
            }
            else {
                driveTrain.driveTo(702,0,90, 200);
           }
            drawing.outtake();
            sleep(10);
            driveTrain.driveTo(350, 0, 0, 200);
            driveTrain.driveTo(0, 0, 0, 200);
            driveTrain.driveTo(0, -400, 0, 200);


        }
    }

}
