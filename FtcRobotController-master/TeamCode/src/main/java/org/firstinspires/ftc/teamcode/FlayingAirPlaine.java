package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class FlayingAirPlaine {
    private Servo AirPlane = null;
    private final LinearOpMode opMode;
    public FlayingAirPlaine (HardwareMap hw, LinearOpMode opMode){
        this.opMode = opMode;
        AirPlane =hw.get(Servo.class ,"AirPlane");

      //  AirPlane.setPosition(Servo.Direction.FORWARD);

    }
    public void Fly(){
        AirPlane.setPosition(1);

    }
    public void Return(){
        AirPlane.setPosition(0);
    }
}
