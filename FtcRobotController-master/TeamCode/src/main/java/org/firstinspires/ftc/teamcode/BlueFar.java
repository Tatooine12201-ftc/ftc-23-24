package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
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
    Lift lift;

    ElapsedTime timer = new ElapsedTime();
    Arm arm;
    private VisionPortal visionPortal;
    OutTake outTake;


    @Override
    public void runOpMode() {
        DriveTrain driveTrain = new DriveTrain(hardwareMap, this);
        Drawing drawing = new Drawing(hardwareMap,this);
        driveTrain.setStartPos(0, 0, 0);
        Arm arm = new Arm(hardwareMap, this);
        Lift lift = new Lift(hardwareMap, this);

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
        if (opModeIsActive() && !isStopRequested()) {
            driveTrain.driveTo(702, 0, 0, 3000);

            drawing.outtake();
            sleep(10);
           driveTrain.driveTo(200, 0, 0, 3000);
            driveTrain.driveTo(840,-900 , -90, 3000);
            lift.setLevel(1);
            sleep(2000);
            arm.pos();
            sleep(2000);
            lift.setLevel(0);
//
//            if (location == Location.MIDDLE) {
//
//
//
//
//            }
//
//          else if (location == Location.LEFT) {
//              driveTrain.driveTo(702, 0,-90,200);
//                // drawing.outtake();
//               // sleep(10);
//                driveTrain.driveTo(200, 0, 0, 200);
//                driveTrain.driveTo(850,-900 , -90, 200);
//                lift.setLevel(1);
//                sleep(2000);
//                arm.pos();
//                sleep(2000);
//                lift.setLevel(0);
//
//
//            }
//            else {
//                driveTrain.driveTo(702,0,90, 200);
//               // drawing.outtake();
//               // sleep(10);
//                driveTrain.driveTo(200, 0, 0, 200);
//                driveTrain.driveTo(830,-900 , -90, 200);
//                lift.setLevel(1);
//                sleep(2000);
//                arm.pos();
//                sleep(2000);
//                lift.setLevel(0);
//
//
//            }

            //   driveTrain.driveTo(0, 0, 0, 200);


        }
    }

}
