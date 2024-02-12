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
public class BlueClose extends LinearOpMode{

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
        visionPortal.stopStreaming();
        if (opModeIsActive() && !isStopRequested()) {
            if (location == ColorDetector.Location.MIDDLE) {
                driveTrain.driveTo(702, 0, 0, 3000);
                drawing.outtake();
                sleep(10);
                driveTrain.driveTo(100, 0, 0, 3000);
                //driveTrain.driveTo(0, -2100, 0, 3000);
                driveTrain.driveTo(0, -2000, 0, 3000);
                driveTrain.driveTo(850, -2100, 0, 3000);
            }
        else if (location == ColorDetector.Location.LEFT) {
                driveTrain.driveTo(702, 0, 90, 3000);
                drawing.outtake();
                sleep(10);
                driveTrain.driveTo(100, 0, 0, 3000);
                driveTrain.driveTo(0, -2000, 0, 3000);
                driveTrain.driveTo(840, -2100, 0, 3000);
            }
                else{
                    driveTrain.driveTo(702, 0, -90, 3000);
                drawing.outtake();
                sleep(10);
                driveTrain.driveTo(100, 0, 0, 3000);
                driveTrain.driveTo(0, -2100, 0, 3000);
                driveTrain.driveTo(0, -2000, 0, 3000);
                lift.setLevel(2);
                sleep(2000);
                arm.pos();
                sleep(1000);
                driveTrain.driveTo(860, -2100, 0, 3000);
                outTake.PutOut1();
                sleep(1000);
                arm.stosStart();
                sleep(2000);
                lift.setLevel(0);

                    }



        }
    }

}


