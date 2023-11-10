package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.Pid;

public class lift {

    private DcMotor LiftMotor = null;

    private Pid pid;
    private  LinearOpMode opMode;

    public static double TICKS_PER_REV = 1150;
    public static double GEAR_RATIO = 1;
    public static double PULY_PERIMITAR = 78.5;
    private static final double COUNTS_PER_MM =GEAR_RATIO* TICKS_PER_REV /PULY_PERIMITAR;
    private final int [] levels ={5,7};
    double F = 0;

    private int level = 0;
    public lift(HardwareMap hw,LinearOpMode opMode) {
        this.opMode=opMode;

        LiftMotor=hw.get(DcMotorEx.class,"LiftMotor");

        LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        resetEncoders();

        stopm();

        pid = new Pid(0,00,0,0);

        F = pid.getF();

        pid.setIntegrationBounds(0,0);
        pid.setTolerance(0);

    }


    public double ticksToMM(double ticks) {
        return ticks / COUNTS_PER_MM;
    }
    public double MMToTicks(double MM){return MM * COUNTS_PER_MM;}

    private void stopm() {
        LiftMotor.setPower(0);

    }
    public lift(DcMotor lift) {
        this. LiftMotor = lift;
    }

    public int getLevel() {
        return this.level;
    }

    public void setLevel(int level) {
        if (level >= 0 && level <= 7) {
            this.level = level;
        }
    }

    public int getEncoder() {
        return LiftMotor.getCurrentPosition();
    }
    public void reset() {
        resetEncoders();
    }
    public void resetEncoders() {
        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }






    }
