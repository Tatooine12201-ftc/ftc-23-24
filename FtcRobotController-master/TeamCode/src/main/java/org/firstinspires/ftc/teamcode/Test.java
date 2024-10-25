package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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

    OpMode opMode;


    ElapsedTime timer = new ElapsedTime();
    private int count = 0;
    boolean IsUp = false;


    @Override
    public void runOpMode() {
        DriveTrain driveTrain = new DriveTrain(hardwareMap, this);
        Lift lift = new Lift(hardwareMap, this);
        Arm arm = new Arm(hardwareMap, this);
        driveTrain.setStartPos(0, 0, 0);
        OutTake outTake = new OutTake(hardwareMap, this);

        waitForStart();
        lift.reset();
       while (opModeIsActive() && !isStopRequested()) {
        lift.GoTo();

         telemetry.addData("encoder1 ",lift.getEncoder());
         telemetry.update();
    }
    }
}
