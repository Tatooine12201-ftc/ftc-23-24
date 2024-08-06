package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.Gamepad;
@TeleOp(name = "teleop1", group = "OpMode")
public class TeleOPTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();
        TestJoyStick testJoyStick = new TestJoyStick(hardwareMap, this);
        while (opModeIsActive() && !isStopRequested()) {
            testJoyStick.On(-gamepad1.left_stick_y);
        }
        }
    }

