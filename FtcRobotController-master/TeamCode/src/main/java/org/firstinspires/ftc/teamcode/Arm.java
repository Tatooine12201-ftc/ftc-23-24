package org.firstinspires.ftc.teamcode;

import android.icu.text.Transliterator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
//import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Pid;

public class Arm {

     private Servo Arm =null;
     private Servo ArmTwo = null;
     private final LinearOpMode opMode;
     Pid pid ;
     AnalogInput analogInput;
    AnalogInput analogInput2;
    private final int [] levels={0,60};
    private int level =0;

    private static final double drive_gearRotation = 1.0/1.0;
    private static final double ServoDegreeArmDegree = drive_gearRotation * 360;


   public Arm (HardwareMap hw,LinearOpMode opMode)
   {
        this.opMode = opMode;

         Arm = hw.get(Servo.class ,"Arm");
         ArmTwo = hw.get(Servo.class, "ArnTwo");
         //analogInput = hw.get(AnalogInput.class, "analogInput");
       //analogInput2 = hw.get(AnalogInput.class, "analogInput2");


        Arm .setDirection(Servo.Direction.REVERSE);
      //  Arm.setPosition(1);
        ArmTwo.setDirection(Servo.Direction.FORWARD);
       // ArmTwo.setPosition(1);

       pid=new Pid(0.3,0,0,0.2);
       pid.setTolerance(0);

       pid.setIntegrationBounds(0,0);





   }
   public int getlevel(){
       return this.level;
   }
    public void setLevel(int level){
        if (level>=0 && level<=1){
            this.level=level;
        }
    }
   public void arm (){
       double armPower=0;
       double armTwoPower=0;
       armPower=pid.calculate(analogInput.getMaxVoltage(),levels[level]);
        Arm.setPosition(armPower);
        ArmTwo.setPosition(armPower);
        opMode.telemetry.addData("armp1",armPower);
        //opMode.telemetry.addData("armTwo", armTwoPower);
        opMode.telemetry.update();

   }

    public double getpos(){
      return Arm.getPosition() ;


    }


    public double getPosition(){
       return analogInput.getVoltage();
    }
    public double getPosition2(){
        return analogInput2.getVoltage();
    }


   public void pos()
   {
       Arm.setPosition(1);
       ArmTwo.setPosition(1);
   }
   public void InteSet(double Inteset){
       Arm.setPosition(Inteset);
       Arm.setPosition(Inteset);
   }

   public void  stosStart (){
    Arm.setPosition(0);
    ArmTwo.setPosition(0);
   }
   public  void checkStop(){
       boolean is_in=false;
        if(getpos()==0){
            is_in=true;
        }
   }


 }
