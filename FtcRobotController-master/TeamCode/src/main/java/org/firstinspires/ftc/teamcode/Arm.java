package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
//import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {

    // private AnalogInput arm ;
     private Servo Arm =null;
     private Servo ArmTwo = null;
     private final LinearOpMode opMode;
     AnalogInput analogInput;

   public Arm (HardwareMap hw,LinearOpMode opMode)
   {
        this.opMode = opMode;
        //AnalogInput arm = hw.get(AnalogInput.class, "arm");
         Arm = hw.get(Servo.class ,"Arm");
         ArmTwo = hw.get(Servo.class, "ArnTwo");

        Arm .setDirection(Servo.Direction.REVERSE);
      //  Arm.setPosition(1);
        ArmTwo.setDirection(Servo.Direction.FORWARD);
      //  ArmTwo.setPosition(1);
       //get our analog input from the hardwareMap

// get the voltage of our analog line
// divide by 3.3 (the max voltage) to get a value between 0 and 1
// multiply by 360 to convert it to 0 to 360 degrees
       double position = analogInput.getVoltage() / 3.3 * 360;
   }

  // public double posAnalog()
  // {
      // double position = arm.getVoltage() / 3.3 * 360;
      // return position;
  // }
    public double getpos(){
      return Arm.getPosition();

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
