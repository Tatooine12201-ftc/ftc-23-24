package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.DriveTrain;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Test")
public class OtonomusTest extends LinearOpMode{

    DriveTrain driveTrain;

    private boolean isRuning() {
        return opModeIsActive() && !isStopRequested();
    }

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        DriveTrain driveTrain = new DriveTrain(hardwareMap, this);
        driveTrain.setStartPos(0,0,0);
        waitForStart();
        driveTrain.reset();

       // driveTrain.driveTo(1000, 1000, 90, 100000);
        driveTrain.driveTo(1000, 1000, 90, 100000);
        sleep(1000);
        driveTrain.driveTo(0, 0, 0, 100000);
            //.driveTo(0, 1000, 0, 10000);
        }




        /*
        while (timer.seconds() < 10) {
            driveTrain.driveTo(200.0, 200.0, 90.0,0);
            if(timer.seconds()> 10){
                driveTrain.driveTo(10 , 10 , 0,0);
            }
        }*/

}
