package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Drawing {
    private DcMotor drawing = null;
    private final LinearOpMode opMode;
    public Drawing(HardwareMap hw, LinearOpMode opMode) {
        this.opMode = opMode;
        drawing = hw.get(DcMotor.class, "drawing");
    }
    public void Power(double power){
        drawing.setPower(power);
    }
    public void stop (){
        drawing.setPower(0);

    }

}