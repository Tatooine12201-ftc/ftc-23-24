package org.firstinspires.ftc.teamcode;

import android.util.Size;

import androidx.annotation.AttrRes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
@Autonomous(name = "BlueParkClose")
public class BluePark extends LinearOpMode {
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
        if (opModeIsActive() &&!isStopRequested()){
            driveTrain.driveTo(70, 0, 0, 4500);
            driveTrain.driveTo(70, 0, -90, 4500);
            driveTrain.driveTo(70, -2000, -90, 4500);
               driveTrain.driveTo(70, -2100, -90, 2000);
               driveTrain.driveTo(180, -2100, -90, 2000);
                driveTrain.driveTo(180, -2100, 0, 2000);
        }

    }
}
