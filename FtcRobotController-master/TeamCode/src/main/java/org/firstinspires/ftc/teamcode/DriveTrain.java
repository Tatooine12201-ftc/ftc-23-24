package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveTrain {
    private DcMotor RFM = null;
    private DcMotor RBM = null;
    private DcMotor LFM = null;
    private DcMotor LBM = null;
    private final LinearOpMode opMode;
    private IMU imu;
    private IMU.Parameters myIMUparameters;
    //private IMU.Parameters RevHubOrientationOnRobot;
    private double heading;


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

        imu.initialize(parameters);

    }

    public void ResetAngle() {
        imu.resetYaw();
    }

    public void Drive(double X, double Y,double RX){
        heading= imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = X * Math.cos(-heading) - Y * Math.sin(-heading);
        double rotY = X * Math.sin(-heading) + Y * Math.cos(-heading);

        //double rotX = X;
        //double rotY = Y;

        rotX = rotX * 1.1;  // Counteract imperfect strafing

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
    }
}

