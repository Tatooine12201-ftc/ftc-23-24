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

public class OutTake {
    private Servo OTS = null;
    private final LinearOpMode opMode;
    public OutTake (HardwareMap hw, LinearOpMode opMode) {
        this.opMode = opMode;
        OTS = hw.get(Servo.class, "Prika");
        OTS.setDirection(Servo.Direction.FORWARD);
    }

    public void PutOut2(){
       OTS.setPosition(1);
    }
    public void PutIn() {OTS.setPosition(-1);}
    public void PutOut1() {OTS.setPosition(0.5);}

 public void Stop(){
        OTS.setPosition(0);
    }
}

