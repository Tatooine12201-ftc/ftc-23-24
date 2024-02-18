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
    ColorDetector colorDetector = new ColorDetector(telemetry, false);
    Drawing drawing;
    Camera camera;
    Location location;
    Lift lift;

    ElapsedTime timer = new ElapsedTime();
    Arm arm;
    private VisionPortal visionPortal;
    OutTake outTake;
    private int count =0;


    @Override
    public void runOpMode() {
        DriveTrain driveTrain = new DriveTrain(hardwareMap, this);
        Drawing drawing = new Drawing(hardwareMap, this);
        driveTrain.setStartPos(0, 0, 0);
        Arm arm = new Arm(hardwareMap, this);
        Lift lift = new Lift(hardwareMap, this);
        OutTake outTake =new OutTake(hardwareMap,this);

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(800, 448));
        builder.enableLiveView(true);
        builder.addProcessor(colorDetector);
        visionPortal = builder.build();




        driveTrain.reset();
        while (opModeInInit()) {
            location = colorDetector.getLocation();

            sleep(30);
        }
        visionPortal.setProcessorEnabled(colorDetector, false);
        if (opModeIsActive() && !isStopRequested()) {


//
            if (location == Location.MIDDLE) {
                driveTrain.driveTo(650, -70, 0, 3000);
                drawing.outtakeoTO();
                sleep(3000);
                drawing.stop();
                driveTrain.driveTo(650, 0, 0, 3000);
                driveTrain.driveTo(300, 50, 0, 3000);

                // lift.setLevel(1);
                sleep(1000);
                // arm.pos();
                sleep(1000);
                driveTrain.driveTo(680, 780, 90, 4000);
                // outTake.PutOut1();
                driveTrain.driveTo(680, 700, 90, 4000);
                driveTrain.driveTo(680, 800, 90, 3000);

                while (count<16){
                    lift.setLevel(1);
                    lift.move();
                    count++;
                }
                sleep(2000);
                arm.pos();
                sleep(2000);
                outTake.PutOut1();
                sleep(2000);
                driveTrain.driveTo(680, 760, 90, 3000);
                arm.stosStart();
                sleep(1000);
                lift.setLevel(0);
                lift.move();


            } else if (location == Location.LEFT) {
                //   driveTrain.driveTo(0, -70, 0, 3000);
                driveTrain.driveTo(0, 300, 0, 2000);
                driveTrain.driveTo(500, 300, 0, 2000);
                //drawing.outtakeoTO();
                //sleep(3000);
                driveTrain.driveTo(100, 150, 0, 3000);
                driveTrain.driveTo(680, 780, 90, 4000);
                driveTrain.driveTo(680, 800, 90, 3000);

                while (count<16){
                    lift.setLevel(1);
                    lift.move();
                    count++;
                }
                sleep(2000);
                arm.pos();
                sleep(2000);
                outTake.PutOut1();
                sleep(2000);
                driveTrain.driveTo(680, 760, 90, 3000);
                arm.stosStart();
                sleep(1000);
                lift.setLevel(0);
                lift.move();
            }


            else {

                driveTrain.driveTo(650, 0, 0, 1000);
                driveTrain.driveTo(630, 30, 0, 1000);
                driveTrain.driveTo(630, 30, 90, 1000);
                driveTrain.driveTo(630, -52, 90, 500);
                sleep(3000);
                drawing.stop();
                driveTrain.driveTo(50, 50, 0, 2000);
                sleep(2000);
                sleep(2000);
                driveTrain.driveTo(680, 800, 90, 3000);

                while (count<16){
                    lift.setLevel(1);
                    lift.move();
                    count++;
                }
                sleep(2000);
                arm.pos();
                sleep(2000);
                outTake.PutOut1();
                sleep(2000);
                driveTrain.driveTo(680, 760, 90, 3000);
                arm.stosStart();
                sleep(1000);
                lift.setLevel(0);
                lift.move();



            }

        }
    }
}
