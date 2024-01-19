package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;


public class OutTake {
    private CRServo OTS = null;
    private final LinearOpMode opMode;
    public OutTake (HardwareMap hw, LinearOpMode opMode) {
        this.opMode = opMode;
        OTS = hw.get(CRServo.class, "Prika");
        OTS.setDirection(CRServo.Direction.FORWARD);
    }

    public void PutOut2(){
       OTS.setPower(-1);
    }
    public void PutIn() {OTS.setPower(1);}
    public void PutOut1() {OTS.setPower(-0.5);}

 public void Stop(){
        OTS.setPower(0);
    }
}

