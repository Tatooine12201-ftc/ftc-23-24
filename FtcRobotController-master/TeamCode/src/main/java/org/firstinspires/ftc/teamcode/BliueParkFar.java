package org.firstinspires.ftc.teamcode;

import android.util.Size;

import androidx.annotation.AttrRes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
@Autonomous(name = "BlueParkFar")
public class BliueParkFar extends LinearOpMode {
    DriveTrain driveTrain;

    Drawing drawing;
    Camera camera;

    Lift lift;

    ElapsedTime timer = new ElapsedTime();
    Arm arm;
    private VisionPortal visionPortal;
    OutTake outTake;

    private int count = 0;

    @Override
    public void runOpMode()  {
        DriveTrain driveTrain = new DriveTrain(hardwareMap, this);
        Drawing drawing = new Drawing(hardwareMap, this);
        driveTrain.setStartPos(0, 0, 0);
        Arm arm = new Arm(hardwareMap, this);
        Lift lift = new Lift(hardwareMap, this);
        driveTrain.resetAngle();
        driveTrain.reset();
        waitForStart();
        if (opModeIsActive() &&!isStopRequested()){
            driveTrain.driveTo(640, 0, 0, 10000);
            driveTrain.driveTo(640, -680, 0, 10000);

        }

    }
}
