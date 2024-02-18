package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ColorDetector.Location;
import org.firstinspires.ftc.vision.VisionPortal;
@Autonomous(name = "Test")
public class Test extends LinearOpMode {


    DriveTrain driveTrain;


    ElapsedTime timer = new ElapsedTime();


    @Override
    public void runOpMode() {
        DriveTrain driveTrain = new DriveTrain(hardwareMap, this);
        Lift lift= new Lift(hardwareMap,this);
        Arm arm = new Arm(hardwareMap,this);
        driveTrain.setStartPos(0, 0, 0);



        driveTrain.reset();
        while (opModeInInit()) {


                while (opModeIsActive() && !isStopRequested()) {


                //driveTrain.driveTo(1000, 0, 0, 30000000);
               // driveTrain.driveTo(0, 0, 90, 30000000);
                    //  driveTrain.driveTo(0,0,90,3000000);
                    driveTrain.Drive(0, 0.20, 0 );
              //       lift.setLevel(1);
             //        lift.move();

              //      arm.pos();


            }
        }
    }
}