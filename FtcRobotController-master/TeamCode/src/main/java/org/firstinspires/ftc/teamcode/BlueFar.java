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
public class BlueFar extends LinearOpMode {

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

    private int count = 0;


    @Override
    public void runOpMode() {
        DriveTrain driveTrain = new DriveTrain(hardwareMap, this);
        Drawing drawing = new Drawing(hardwareMap, this);
        driveTrain.setStartPos(0, 0, 0);
        Arm arm = new Arm(hardwareMap, this);
        Lift lift = new Lift(hardwareMap, this);
        OutTake outTake = new OutTake(hardwareMap, this);


        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(800, 448));
        builder.enableLiveView(true);
        builder.addProcessor(colorDetector);
        visionPortal = builder.build();
        driveTrain.resetAngle();

        driveTrain.reset();
        while (opModeInInit()) {
            location = colorDetector.getLocation();
            // driveTrain.resetAngle();
            sleep(30);
        }
        visionPortal.setProcessorEnabled(colorDetector, false);
        if (opModeIsActive() && !isStopRequested()) {

            if (location == Location.MIDDLE) {
                driveTrain.driveTo(730,-110, 0, 2500);
                drawing.outtakeoTO();
                sleep(1500);
                drawing.stop();
                driveTrain.driveTo(0, -20, 0, 2500);
                driveTrain.driveTo(640, -620, 0, 2500);
                driveTrain.driveTo(640, -620, 100, 2500);
                timer.reset();
                timer.startTime();
                if (timer.milliseconds()<0.5){
                    lift.GoTo();
                }
                arm.pos7();

                sleep(2000);
                driveTrain.driveTo(640,-880,100,2000);
                outTake.PutOut1();
                sleep(1000);
                driveTrain.driveTo(650, -660, 100, 2500);
                arm.stosStart();
                sleep(2000);
                outTake.Stop();
                timer.reset();
                while (timer.seconds()<4) {
                    lift.lift_t(-1);
                }
                sleep(2000);


            } else if (location == Location.LEFT) {


                driveTrain.driveTo(300, -300, 0, 1500);
                driveTrain.driveTo(500, -300, 0, 1500);
                drawing.outtake();
                sleep(2000);
                drawing.stop();
                driveTrain.driveTo(100, -260, 0, 1500);
                driveTrain.driveTo(0, -20, 0, 2500);
                driveTrain.driveTo(520, -620, 0, 3000);
                driveTrain.driveTo(520, -620, 90, 2500);
                timer.reset();
                timer.startTime();
                if (timer.milliseconds()<0.5){
                    lift.GoTo();
                }
                arm.pos7();

                sleep(2000);
                driveTrain.driveTo(540,-880,100,2000);
                outTake.PutOut1();
                sleep(1000);
                driveTrain.driveTo(500, -680, 100, 2500);
                arm.stosStart();
                sleep(2000);
                outTake.Stop();
                timer.reset();
                while (timer.seconds()<4) {
                    lift.lift_t(-1);
                }
                sleep(2000);


                }



             else{

                driveTrain.driveTo(540, 0, 0, 2000);
                driveTrain.driveTo(540, 0, 45, 2000);
                driveTrain.driveTo(540, 200, 45, 2000);
                drawing.outtakeo3();
                driveTrain.driveTo(540, 0, 45, 2000);
                driveTrain.driveTo(40,-90,0,2000);
                drawing.stop();
                driveTrain.driveTo(50, -50, 0, 2000);
                driveTrain.driveTo(800, -620, 0, 3000);
                driveTrain.driveTo(800, -620, 90, 2500);

                timer.reset();
                timer.startTime();
                if (timer.milliseconds()<0.5){
                    lift.GoTo();
                }
                arm.pos7();

                sleep(2000);
                driveTrain.driveTo(800,-890,100,2000);
                outTake.PutOut1();
                sleep(1000);
                driveTrain.driveTo(820, -680, 100, 2500);
                arm.stosStart();
                sleep(2000);
                outTake.Stop();
                timer.reset();
                while (timer.seconds()<4) {
                    lift.lift_t(-1);
                }
                sleep(2000);
                }
            }
        }


    }
