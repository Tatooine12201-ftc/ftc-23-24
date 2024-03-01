package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ColorDetector.Location;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Timer;

@Autonomous(name = "Test")
public class Test extends LinearOpMode {


    DriveTrain driveTrain;
    OutTake outTake;


    ElapsedTime timer = new ElapsedTime();
    private int count = 0;
    boolean IsFed = false;

    @Override
    public void runOpMode() {
        DriveTrain driveTrain = new DriveTrain(hardwareMap, this);
        Lift lift = new Lift(hardwareMap, this);
        Arm arm = new Arm(hardwareMap, this);
        driveTrain.setStartPos(0, 0, 0);
        OutTake outTake = new OutTake(hardwareMap,this);


        driveTrain.reset();
        while (opModeInInit()) {


            if (opModeIsActive() && !isStopRequested()) {


                //driveTrain.driveTo(1000, 0, 0, 30000000);
                // driveTrain.driveTo(0, 0, 90, 30000000);
                //  driveTrain.driveTo(0,0,90,3000000);
                timer.startTime();
                while (timer.seconds() < 6) {
                    lift.MoveForAtonomomus();


                }


                 if (count>=6){
                       lift.FForAtonomomus();
                   }
                arm.pos();
                sleep(2000);
                outTake.PutOut1();
                arm.stosStart();
                sleep(2000);
                  /*
                   sleep(2000);
                   arm.pos();
                   sleep(3000);
                   arm.stosStart();
                   sleep(3000);





                  */
            }
        }
    }
}