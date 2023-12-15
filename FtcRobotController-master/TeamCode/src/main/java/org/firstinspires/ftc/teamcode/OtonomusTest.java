package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.DriveTrain;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "ImBadAtThisShit")
public class OtonomusTest extends LinearOpMode{
    DriveTrain driveTrain;

    private boolean isRuning() {
        return opModeIsActive() && !isStopRequested();
    }

    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() {
        while (timer.seconds() < 10) {
            driveTrain.driveTo(200.0, 200.0, 90.0);
            if(timer.seconds()> 10){
                driveTrain.driveTo(10 , 10 , 0);
            }
        }
    }
}