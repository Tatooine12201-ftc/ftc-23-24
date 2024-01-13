package org.firstinspires.ftc.teamcode;

import android.media.FaceDetector;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DriveTrain;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;


import org.firstinspires.ftc.teamcode.DriveTrain;

 @TeleOp(name = "teleop", group = "OpMode")
    
public class teleop extends LinearOpMode {
        DriveTrain driveTrain;
        Drawing drawing;
        Lift lift;
      //  Arm arm ;
        //OutTake outTake;
        Gamepad gamepad1Old = new Gamepad();
        Gamepad gamepad2Old = new Gamepad();


        @Override
        public void runOpMode() {

            DriveTrain driveTrain = new DriveTrain(hardwareMap, this);
            Drawing drawing = new Drawing(hardwareMap,this);
            Lift lift =new Lift ( hardwareMap ,this);
           // OutTake outTake = new OutTake(hardwareMap, this);
            //Arm arm = new Arm(hardwareMap);

            driveTrain.setStartPos(0,0,0);
            driveTrain.reset();
            waitForStart();

                while (opModeIsActive() && !isStopRequested()) {
                    driveTrain.Drive(-gamepad1.left_stick_y,gamepad1.left_stick_x, gamepad1.right_trigger - gamepad1.left_trigger);
                  //  if (gamepad2.right_bumper) {
                  //      outTake.PutIn();
                   // }
                 //   else {
                   //     outTake.Stop();
                  //  }
               //     if (gamepad2.right_bumper) {
                       // outTake.PutOut();
                   // }
                  //  else {
                    //    outTake.Stop();
              //      }
               if (gamepad2.right_bumper) {
                   drawing.intake();
                    }
               else{
                 drawing.stop();

               }
           //    if (gamepad2.a){
           //        arm.pos();
               //}
              // arm.pos();
                lift.Lift(gamepad2.right_stick_y);

                if (gamepad1.options) {
                    driveTrain.resetAngle();
                }
                driveTrain.update();




               //ערך מינימלי של צירים
                // driveTrain.Drive(0,0.21,0);
                // driveTrain.Drive(0, 0, 0.1);

//                telemetry.addData("m1",driveTrain.getlbm());
//                telemetry.addData("m2",driveTrain.getlfm());
//                telemetry.addData("m3",driveTrain.getrbm());
//                telemetry.addData("m4",driveTrain.getrfm());
                //
                // telemetry.addData("heading",Math.toDegrees(driveTrain.Heading()));

                gamepad1Old = gamepad1;
                gamepad2Old = gamepad2;
                telemetry.update();
            }

        }
    }

