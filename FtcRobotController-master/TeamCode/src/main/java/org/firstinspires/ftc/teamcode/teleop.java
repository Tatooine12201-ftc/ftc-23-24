package org.firstinspires.ftc.teamcode;

import android.media.FaceDetector;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DriveTrain;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DriveTrain;

 @TeleOp(name = "teleop", group = "OpMode")
    
public class teleop extends LinearOpMode {
     DriveTrain driveTrain;
     Drawing drawing;
     //  Lift lift;
     Arm arm;
     OutTake outTake;
     AnalogInput analogInput;
     private boolean isbussy = true;
     TouchSensor touch;
     Gamepad gamepad1Old = new Gamepad();
     Gamepad gamepad2Old = new Gamepad();
     ElapsedTime timer = new ElapsedTime();
     boolean isPressed;


     @Override
     public void runOpMode() {

         DriveTrain driveTrain = new DriveTrain(hardwareMap, this);
         Drawing drawing = new Drawing(hardwareMap, this);
         Lift lift = new Lift(hardwareMap, this);
         OutTake outtake = new OutTake(hardwareMap, this);
         Arm arm = new Arm(hardwareMap, this);


         driveTrain.setStartPos(0, 0, 0);
         driveTrain.reset();
         telemetry.addData("Mode", "waiting");
         telemetry.update();
         waitForStart();
         telemetry.addData("Mode", "waiting");
         telemetry.update();


         while (opModeIsActive() && !isStopRequested()) {
             driveTrain.Drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_trigger - gamepad1.left_trigger);

             if (gamepad2.right_bumper) {
                 drawing.inTake();
                 outtake.PutIn();

             } else if (gamepad2.left_bumper) {
                 outtake.PutOut1();
             } else {
                 outtake.Stop();
                 drawing.stop();
             }

                 if (gamepad2.circle) {
                     drawing.outtake();
                 }

                 if (gamepad2.cross) {
                     arm.stosStart();

                 }

                 if (gamepad2.square) {
                     arm.pos();
                 }

                 lift.lift(gamepad2.right_stick_y);
                 isPressed = gamepad2.right_stick_y > 0.05;
                 if (!isPressed) {
                     lift.setF();
                 }
                 if (gamepad2.triangle) {
                     while (timer.seconds()<0.5){
                         arm.stosStart();
                     }
                     lift.setLevel(0);
                 }


                 if (gamepad1.options) {
                     driveTrain.resetAngle();
                 }
                 driveTrain.update();

                 telemetry.addData("ang", arm.getpos());

                 gamepad1Old = gamepad1;
                 gamepad2Old = gamepad2;
                 telemetry.update();
             }

         }

 }

