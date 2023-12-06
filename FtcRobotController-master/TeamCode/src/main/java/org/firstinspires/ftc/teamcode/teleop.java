package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.DriveTrain;

    @TeleOp(name = "teleop", group = "OpMode")
public class teleop extends LinearOpMode {
        DriveTrain driveTrain;
        Drawing drawing;

        @Override
        public void runOpMode() {
            DriveTrain driveTrain = new DriveTrain(hardwareMap, this);
            Drawing drawing = new Drawing(hardwareMap,this);
            driveTrain.reset();
            waitForStart();

            while (opModeIsActive() && !isStopRequested()) {
                driveTrain.Drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_trigger - gamepad1.left_trigger);
                if (gamepad2.left_bumper){
                    drawing.intake();
                }
                else if (gamepad2.right_bumper){
                    drawing.outtake();
                }
                else {
                    drawing.stop();
                }

                telemetry.addData("m1",driveTrain.getlbm());
                telemetry.addData("m2",driveTrain.getlfm());
                telemetry.addData("m3",driveTrain.getrbm());
                telemetry.addData("m4",driveTrain.getrfm());
                telemetry.addData("heading",driveTrain.Heading());
                telemetry.update();

            }

        }
    }

