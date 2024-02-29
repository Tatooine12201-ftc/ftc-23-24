package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous(name = "ReturnRedCloseStartPoint")
public class ReturnToRedStartPoint extends LinearOpMode {
    DriveTrain driveTrain;

    public void runOpMode() {
        DriveTrain driveTrain = new DriveTrain(hardwareMap, this);
        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            driveTrain.setStartPos(100.0, 2400.0, 0.0);
            driveTrain.driveTo(100 ,2400 ,0,2000);
            driveTrain.driveTo(100 ,2400 ,-90,2000);
            driveTrain.driveTo(100 ,0 ,-90,4000);
            driveTrain.driveTo(100 ,0 ,0,2000);
            driveTrain.driveTo(0 ,0 ,0,2000);

        }
    }
}