package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class DriveTrain {
    private DcMotor RFM = null;
    private DcMotor RBM = null;
    private DcMotor LFM = null;
    private DcMotor LBM = null;
    private final LinearOpMode opMode;
    //  private IMU imu;
    // private IMU.Parameters myIMUparameters;
    //private IMU.Parameters RevHubOrientationOnRobot;
    private double heading;
    private IMU imu = null;
    private static final double TPI = Math.PI * 2;


    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));


    public DriveTrain(HardwareMap hw, LinearOpMode opMode) {
        this.opMode = opMode;
        RFM = hw.get(DcMotor.class, "RFM");
        RBM = hw.get(DcMotor.class, "RBM");
        LFM = hw.get(DcMotor.class, "LFM");
        LBM = hw.get(DcMotor.class, "LBM");
        imu = hw.get(IMU.class, "imu");

        RFM.setDirection(DcMotorSimple.Direction.REVERSE);
        RBM.setDirection(DcMotorSimple.Direction.REVERSE);
        LFM.setDirection(DcMotorSimple.Direction.FORWARD);
        LBM.setDirection(DcMotorSimple.Direction.FORWARD);

        // imu

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);


        RFM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hw.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.initialize(parameters);


    }

    public void ResetAngle() {
        double reset = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }

    public double NornalizeAngl(double angle) {
        while (angle > Math.PI) {
            angle -= TPI;
        }
        while (angle < -Math.PI) {
            angle += TPI;
        }
        return angle;
    }






    public void Drive(double X, double Y, double RX){
        heading = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;

        double rotX = X * Math.cos(heading) - Y * Math.sin(heading);
        double rotY = X * Math.sin(heading) + Y * Math.cos(heading);

        //double rotX = X;
        //double rotY = Y;

       // rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(RX), 1);
        double frontLeftPower = (rotY + rotX + RX) / denominator;
        double backLeftPower = (rotY - rotX + RX) / denominator;
        double frontRightPower = (rotY - rotX - RX) / denominator;
        double backRightPower = (rotY + rotX - RX) / denominator;

        LFM.setPower(frontLeftPower);
        LBM.setPower(backLeftPower);
        RFM.setPower(frontRightPower);
        RBM.setPower(backRightPower);


        opMode.telemetry.addData("Power", LBM.getPower());
        opMode.telemetry.addData("Power", RBM.getPower());
        opMode.telemetry.addData("Power", RFM.getPower());
        opMode.telemetry.addData("Power", LFM.getPower());


    }
    public double getlbm (){
        return LBM.getPower();
    }

    public double getrbm (){
        return RBM.getPower();
    }

    public double getrfm (){
        return  RFM.getPower();
    }
    public double getlfm (){
        return LFM.getPower();
    }

}

