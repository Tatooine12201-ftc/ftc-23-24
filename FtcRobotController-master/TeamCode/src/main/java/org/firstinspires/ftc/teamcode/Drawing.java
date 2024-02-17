package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.teamcode.OutTake;

public class Drawing {
    private DcMotor drawing = null;

    private final LinearOpMode opMode;



    public Drawing(HardwareMap hw, LinearOpMode opMode) {
        this.opMode = opMode;
        drawing = hw.get(DcMotor.class, "drawing");
        drawing.setDirection(DcMotorSimple.Direction.FORWARD);
    }


    public void stop() {
        drawing.setPower(0);
    }

    public void inTake() {

        drawing.setPower(1);
    }
    public void outtake() {

        drawing.setPower(-1);
    }
    public void outtakeoTO() {

        drawing.setPower(-0.3);
    }

}
