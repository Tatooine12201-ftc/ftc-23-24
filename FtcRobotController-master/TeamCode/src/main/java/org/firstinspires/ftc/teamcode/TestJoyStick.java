package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
public class TestJoyStick {
    private DcMotor TestMotor = null;
    private final LinearOpMode opMode;

    public TestJoyStick(HardwareMap hw, LinearOpMode opMode) {
        this.opMode = opMode;
        TestMotor = hw.get(DcMotor.class, "TestMotor");
    }
    public void On (double X){
        TestMotor.setPower(X);

    }
}
