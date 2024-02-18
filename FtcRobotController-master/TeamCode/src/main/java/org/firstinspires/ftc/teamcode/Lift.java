package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
//ticks per revolution
    public static double TICKS_PER_GOBILDA = 312;
    //gear ratio
    public static double GEAR_RATIO = 1.0/1.0;
    //Puly perimitar
    public static double PULLEY_DIAMETER = 20;
    //Calculate Counts Per MM
    private final double COUNTS_PER_MM =( TICKS_PER_GOBILDA * GEAR_RATIO )/ (PULLEY_DIAMETER * Math.PI);
    //Lift Levels
    private final int[] levels = {0,1900,1000};
    // Lift Pid force
    private double KF = 0;
    // Lift Motor
    private DcMotor LiftMotor = null;

    private DcMotor LiftMotortow = null;
    //Lift PID
    private Pid pid;
    //Lift opMode
    private double PlusPower = 0;
    private LinearOpMode opMode;
    //Lift level 0
    private int level = 0;
    private int pos =0;
    private boolean isPowered = false;

    //lift settings
    public Lift(HardwareMap hw, LinearOpMode opMode) {
        this.opMode = opMode;

        LiftMotor = hw.get(DcMotorEx.class, "LiftMotor");
        LiftMotortow =hw.get(DcMotorEx.class, "LiftMotortow");

        LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LiftMotortow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LiftMotortow.setDirection(DcMotorSimple.Direction.REVERSE);

        resetEncoders();

        stop();

        // 0.2
        //pid = new Pid(0.98, 0, 0, 0.9);
        pid = new Pid(0.95, 0, 0, 0.8);

        // KF = pid.getF();


        pid.setIntegrationBounds(-0.17, 0.17);
        pid.setTolerance(0);
        stop();

    }


    //ticks to MM and MM to ticks
    public double ticksToMM(double ticks) {
        return ticks / COUNTS_PER_MM;
    }

    public double MMToTicks(double MM) {
        return MM * COUNTS_PER_MM;
    }

    //stop: set the lift to lvl 0
         public void stop() {
        LiftMotor.setPower(0);
        LiftMotortow.setPower(0);

    }
// get lift lvl and return lift lvl
    public int getLevel() {
        return this.level;
    }

    //set lift lvl to required lvl
    public void setLevel(int level) {
        if (level >= 0 && level <= 2) {
            this.level = level;
        }
    }

    //get Encoders ticks
    public double getEncoder() {
        return  LiftMotor.getCurrentPosition();

    }
    public double getEncoder2() {
        return  -LiftMotortow.getCurrentPosition();

    }

    public double getF(){
        return pid.getF();
    }

    //reset encoders ticks
    public void reset() {
        resetEncoders();
    }

    public void resetEncoders() {
        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LiftMotortow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       LiftMotortow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void lift (double power ){
            LiftMotor.setPower(power);
            LiftMotortow.setPower(power);
        }


    public void setF() {
        if (getEncoder() > 0) {
            pid.setF(0.25);
            LiftMotor.setPower(pid.getF());
            LiftMotortow.setPower(pid.getF());
        }
    }


    public void getpower(double p){
        LiftMotor.setPower(p);
        LiftMotortow.setPower(p);
    }

    public void LiftByTicks(){
          LiftMotor.setTargetPosition(LiftMotor.getCurrentPosition()+1);
          LiftMotortow.setTargetPosition(LiftMotortow.getCurrentPosition()+1);


    }
    public void move (){
        double target =levels[level];
        double out =0;
        double out2 =0;
        out=pid.calculate(getEncoder(),target);
        out2 = pid.calculate(getEncoder2(), target);
        LiftMotor.setPower(out);
        LiftMotortow.setPower(out);
        opMode.telemetry.addData("Encoder1",ticksToMM(getEncoder()));
        opMode.telemetry.addData("Encoder2", ticksToMM(getEncoder2()));
        opMode.telemetry.addData("target", target);
        opMode.telemetry.update();

    }


}
