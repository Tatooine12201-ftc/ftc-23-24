package org.firstinspires.ftc.teamcode;

import android.icu.text.Transliterator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
//import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Pid;

public class Arm {

     private Servo Arm =null;
     private Servo ArmTwo = null;
     private final LinearOpMode opMode;
     private Pid pid ;
     AnalogInput analogInput;
    AnalogInput analogInput2;
     private final Pid pidArm =new Pid(0,0,0,0);
    private final Pid pidArmTwo =new Pid(0,0,0,0);

   public Arm (HardwareMap hw,LinearOpMode opMode)
   {
        this.opMode = opMode;

         Arm = hw.get(Servo.class ,"Arm");
         ArmTwo = hw.get(Servo.class, "ArnTwo");

        Arm .setDirection(Servo.Direction.REVERSE);
      //  Arm.setPosition(1);
        ArmTwo.setDirection(Servo.Direction.FORWARD);
      //  ArmTwo.setPosition(1);
       pidArm.setTolerance(0);
       pidArmTwo.setTolerance(0);
       pidArm.setIntegrationBounds(0,0);
       pidArmTwo.setTolerance(0);
       pidArmTwo.setIntegrationBounds(0,0);



   }
    double position = analogInput.getVoltage() / 3.3 * 360;
    double position2 = analogInput2.getVoltage() / 3.3 * 360;
   public void arm (){
       double armPower=0;
       double armTwoPower=0;
       armPower=pidArm.calculate(position,Arm.getPosition());
       armTwoPower=pidArmTwo.calculate(position2,ArmTwo.getPosition());
   }
    public double getpos(){
      return Arm.getPosition() ;


    }

    public double getPosition(){
       return position ;
    }
    public double getPosition2(){
        return position2 ;
    }


   public void pos()
   {
       Arm.setPosition(0.7);
       ArmTwo.setPosition(0.7);
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
