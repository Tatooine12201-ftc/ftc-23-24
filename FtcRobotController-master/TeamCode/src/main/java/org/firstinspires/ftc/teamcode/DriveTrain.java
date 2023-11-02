package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.max;
import static java.lang.Math.min;
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
    public static double FORWARD_OFFSET = 20.97;

    double prevRightEncoderPos = 0;
    double prevLeftEncoderPos = 0;
    double prevCenterEncoderPos = 0;
    public static double TICKS_PER_REV = 8192;
    public static double GEAR_RATIO = 1;
    public static double WHEEL_DIAMETER = 35;
    boolean wasReset = false;
    private double robotHading_CWP = 0;
    private double robotHading_CCWP = 0;
    private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    private double fieldX = 0;
    double reset = 0;
    private static final double COUNTS_PER_MM = (TICKS_PER_REV * GEAR_RATIO) / WHEEL_CIRCUMFERENCE;
    private double fieldY = 0;
    public static double LATERAL_DISTANCE = 127.5;
    private static final double TPI = Math.PI * 2;

    public double startX = 0;
    public double startY = 0;
    public double startR = 0;


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
    public void reset() {
        //reset the encoders that are used for Xr, Xl and Y
        LFM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RFM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LFM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //reset the imu (cw is positive)
        reset = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;

    }

    public void setAngle(double angle, boolean x) {
        //set the angle of the robot
        if (x && !wasReset) {
            reset = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle - angle;
            wasReset = true;
        } else {
            wasReset = false;
        }
    }
    public double NormalizeAngle(double angle) {
        while (angle > Math.PI) {
            angle -= TPI;
        }
        while (angle < -Math.PI) {
            angle += TPI;
        }
        return angle;
    }
    public double Heading() {
        //return the heading of the robot (ccw is positive) in radians (0 to 2pi) and make sure that the start R is taken into account
        robotHading_CWP = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle + startR - reset; //cw is positive
        robotHading_CCWP = -robotHading_CWP; //ccw is positive
        return NormalizeAngle(robotHading_CCWP); //normalize the angle to be between -pi and pi
    }
    public double getXlEncoder() {
        return -LBM.getCurrentPosition();
    }
    public double getXrEncoder() {
        return -RFM.getCurrentPosition();
    }
    public double getYEncoder() {
        return LFM.getCurrentPosition();
    }
    public double ticksToMM(double ticks) {
        return ticks / COUNTS_PER_MM;
    }
    private void update() {
        //save the Encoder position
        double leftEncoderPos = getXlEncoder();
        double rightEncoderPos = getXrEncoder();
        double centerEncoderPos = getYEncoder();

        //calculate the change in encoder position from the previous iteration of the loop
        double deltaLeftEncoderPos = leftEncoderPos - prevLeftEncoderPos;
        double deltaRightEncoderPos = rightEncoderPos - prevRightEncoderPos;
        double deltaCenterEncoderPos = centerEncoderPos - prevCenterEncoderPos;

        //calculate the change in position of the robot
        double phi = (deltaLeftEncoderPos - deltaRightEncoderPos) / LATERAL_DISTANCE;
        double deltaMiddlePos = (deltaLeftEncoderPos + deltaRightEncoderPos) / 2;
        double deltaPerpPos = deltaCenterEncoderPos - FORWARD_OFFSET * phi;

        //calculate the change in the field position of the robot
        double heading = robotHading_CCWP;
        double deltaX = deltaMiddlePos * cos(heading) + deltaPerpPos * sin(heading);
        double deltaY = -deltaMiddlePos * sin(heading) + deltaPerpPos * cos(heading);

        //update the field position of the robot
        fieldX += ticksToMM(deltaX);
        fieldY += ticksToMM(deltaY);
        Heading();

        //save the encoder position for the next iteration of the loop
        prevLeftEncoderPos = leftEncoderPos;
        prevRightEncoderPos = rightEncoderPos;
        prevCenterEncoderPos = centerEncoderPos;

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
    public void setStartPos(double x, double y, double r) {
        //set the start position
        startX = x;
        startY = y;
        startR = Math.toRadians(r);
        //set the field position
        fieldX = x;
        fieldY = y;
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

