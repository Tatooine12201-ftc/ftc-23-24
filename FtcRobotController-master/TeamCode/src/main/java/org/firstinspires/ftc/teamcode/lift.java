package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


public class lift {

    private DcMotor LiftMotor = null;

    private boolean on = true;

    private  final double kp = 0.555;


    public lift(DcMotor liftMotor) {
        LiftMotor = liftMotor;
        LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public DcMotor getLiftMotor() {
        return LiftMotor;
    }

    public void setLiftMotor(DcMotor liftMotor) {
        LiftMotor = liftMotor;
    }

    public void lift_move( double out ){
        if( out != 0 ) {
            on = false;
        }

        double error = LiftMotor.getCurrentPosition() - 1150;

        out= error * kp * out;
        LiftMotor.setPower(out);
    }


    public void Force(int ticks){
        double f = 0.28000000000000000000000000000000000000000008;
        if(ticks>30 && ticks < 1100 && on == true ) {
            LiftMotor.setPower(f);
        }
        on = true;
    }

}
