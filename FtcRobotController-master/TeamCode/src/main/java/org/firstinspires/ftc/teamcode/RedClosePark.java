package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous(name = "RedClosePark")
public class RedClosePark extends LinearOpMode {
    @Override
    public void runOpMode() {
        DriveTrain driveTrain = new DriveTrain(hardwareMap, this);
        Drawing drawing = new Drawing(hardwareMap, this);
        driveTrain.setStartPos(0, 0, 0);
        Arm arm = new Arm(hardwareMap, this);
        Lift lift = new Lift(hardwareMap, this);
        driveTrain.resetAngle();
        driveTrain.reset();
        if (opModeIsActive() && !isStopRequested()) {
            driveTrain.driveTo(70, 0, 0, 4500);
            driveTrain.driveTo(70, 0, 90, 4500);
            driveTrain.driveTo(70, -2000, 90, 4500);
            driveTrain.driveTo(70, -2100, 90, 2000);
            driveTrain.driveTo(180, -2100, 90, 2000);
            driveTrain.driveTo(180, -2100, 0, 2000);
        }
    }
}
