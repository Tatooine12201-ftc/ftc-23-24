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
    public static double GEAR_RATIO = 53.0/13.0;
    //Puly perimitar
    public static double PULLEY_DIAMETER = 20;
    //Calculate Counts Per MM
    private final double COUNTS_PER_MM = TICKS_PER_GOBILDA * GEAR_RATIO / PULLEY_DIAMETER * Math.PI;
    //Lift Levels
    private final int[] levels = {5, 7};
    // Lift Pid force
    private double KF = 0;
    // Lift Motor
    private DcMotor LiftMotor = null;

    private DcMotor LiftMotortow;
    //Lift PID
    private Pid pid;
    //Lift opMode
    private double PlusPower = 0;
    private LinearOpMode opMode;
    //Lift level 0
    private int level = 0;
    private int pos =0;

    //lift settings
    public Lift(HardwareMap hw, LinearOpMode opMode) {
        this.opMode = opMode;

        LiftMotor = hw.get(DcMotorEx.class, "LiftMotor");
        LiftMotortow =hw.get(DcMotorEx.class, "LiftMotortow");

        LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LiftMotortow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LiftMotortow.setDirection(DcMotorSimple.Direction.FORWARD);

        resetEncoders();

        stop();

        pid = new Pid(0, 0, 0, 0.55555569);

        KF = pid.getF();

        pid.setIntegrationBounds(0, 0);
        pid.setTolerance(0);

    }

    //setter
    public Lift(DcMotor lift, DcMotor lifttwo) {
        this.LiftMotor = lift;
         this.LiftMotortow = lift;
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
        if (level >= 0 && level <= 7) {
            this.level = level;
        }
    }

    //get Encoders ticks
    public int getEncoder() {
        return LiftMotor.getCurrentPosition();

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
   // public void lift0(){
      //  double Power1 =LiftMotor.getPower();
     //   LiftMotor.setPower(-Power1);
  //  }

    public void setpos(){
        LiftMotor.setTargetPosition(0);
        LiftMotortow.setTargetPosition(0);  }
    public void lift (double power ){

        LiftMotor.setPower(power);
        LiftMotortow.setPower(power);
    }
    public void LiftByTicks(){
          LiftMotor.setTargetPosition(LiftMotor.getCurrentPosition()+1);
          LiftMotortow.setTargetPosition(LiftMotortow.getCurrentPosition()+1);


    }


}
