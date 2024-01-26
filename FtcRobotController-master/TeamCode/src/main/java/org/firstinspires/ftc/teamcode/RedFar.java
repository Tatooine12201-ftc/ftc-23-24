package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "RedFar")
public class RedFar extends LinearOpMode{

    DriveTrain driveTrain;
    ColorDetector colorDetector;
    Drawing drawing;


    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        DriveTrain driveTrain = new DriveTrain(hardwareMap, this);
        Drawing drawing = new Drawing(hardwareMap,this);
        driveTrain.setStartPos(0, 0, 0);
        drawing.inTake();
        driveTrain.reset();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            //  if (אמצע){
            driveTrain.driveTo(-702, 0, 0, 200);
            // }
            //  else if (ימין) {
            driveTrain.driveTo(-702, 0,-90,200);
            // }
            // else {
            driveTrain.driveTo(-702,0,90, 200);
            // }
            drawing.outtake();
            sleep(10);
            driveTrain.driveTo(-350, 0, 0, 200);
            driveTrain.driveTo(0, 850, 0, 200);


        }
    }

}