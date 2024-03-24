package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "teleop", group = "OpMode")

public class teleop extends LinearOpMode {
    private static final double KF = 0;
    DriveTrain driveTrain;
    Drawing drawing;
    //  Lift lift;
    Arm arm;
    OutTake outTake;
    AnalogInput analogInput;
    FlayingAirPlaine flayingAirPlaine;
    TouchSensor touch;
    Gamepad gamepad1Old = new Gamepad();
    Gamepad gamepad2Old = new Gamepad();
    ElapsedTime timer = new ElapsedTime();
    boolean isPressed;
    boolean isArmUp = false;
    boolean IsLiftDown = true;
    boolean IsSlow = false;
    private final boolean isbussy = true;

    @Override
    public void runOpMode() {

        DriveTrain driveTrain = new DriveTrain(hardwareMap, this);
        Drawing drawing = new Drawing(hardwareMap, this);
        Lift lift = new Lift(hardwareMap, this);
        OutTake outtake = new OutTake(hardwareMap, this);
        Arm arm = new Arm(hardwareMap, this);
        FlayingAirPlaine flayingAirPlaine = new FlayingAirPlaine(hardwareMap, this);

        driveTrain.setStartPos(0, 0, 0);
        driveTrain.reset();
        telemetry.addData("Mode", "waiting");
        telemetry.update();
        arm.stosStart();
        waitForStart();
        telemetry.addData("Mode", "waiting");
        telemetry.update();


        while (opModeIsActive() && !isStopRequested()) {
            IsSlow = gamepad1.square;
            if (!IsSlow) {
                driveTrain.Drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_trigger - gamepad1.left_trigger);
            } else {
                driveTrain.DriveSlow(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_trigger - gamepad1.left_trigger);
            }

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
                //    isArmUp = false;

            }
            else if (gamepad2.square) {
                arm.pos();
                //   isArmUp = true;
            }
            else if (gamepad2.triangle) {
                arm.pos7();
            }

            lift.lift(-gamepad2.right_stick_y);

          //  if (gamepad2.triangle) {
          //      lift.setLevel(1);
          //      lift.move();

          //  }


            if (gamepad2.dpad_down) {
              lift.lift(-1);
                }


            if (gamepad2.dpad_up) {
                lift.setLevel(3);
                lift.move();
                //   IsLiftDown = false;
            }
            if (gamepad2.left_stick_button) {
                flayingAirPlaine.Fly();
            } else {
                flayingAirPlaine.Return();
            }
            telemetry.addData("le", lift.getEncoder());
            telemetry.update();


            // isPressed = gamepad2.right_stick_y > 0.05;
            // if (!isPressed) {
            //    lift.setF();
            //  }

            //   if (gamepad2.triangle) {
            //    while (timer.seconds()<0.5){
            //   arm.stosStart();
            // }
            //  lift.setLevel(0);
            //  }


            if (gamepad1.options) {
                driveTrain.resetAngle();
            }
            driveTrain.update();

            //lift.getpower(0.25);


            telemetry.addData("ang", arm.getpos());

            gamepad1Old = gamepad1;
            gamepad2Old = gamepad2;
            telemetry.update();
        }

    }

}

