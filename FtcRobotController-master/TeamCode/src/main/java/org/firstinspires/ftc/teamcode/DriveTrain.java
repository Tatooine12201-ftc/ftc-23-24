package org.firstinspires.ftc.teamcode;


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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class DriveTrain {


    public static IMU imu ;

    private DcMotor RFM = null;
    private DcMotor RBM = null;
    private DcMotor LFM = null;
    private DcMotor LBM = null;
    private final LinearOpMode opMode;


   private double robotHading_CWP = 0;
    private double robotHading_CCWP = 0;


    public static double LATERAL_DISTANCE = 334.67; //המרחק בין האינקודר הימני לשמאלי MM?
    public static double FORWARD_OFFSET = 156.75; //המרחק בין האינקודר X לבין ציר הסיבוב (יותר קרוב למאחורה- שלישי, יותר קרוב למקדימה- חיובי) MM?

    double prevRightEncoderPos = 0;
    double prevLeftEncoderPos = 0;
    double prevCenterEncoderPos = 0;


    public static double TICKS_PER_GOBILDA = 312;
    public static double GEAR_RATIO = 1;
    public static double WHEEL_DIAMETER = 35;


    private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    private double fieldX = 0;
    private double fieldY = 0;


    private static final double COUNTS_PER_MM = (TICKS_PER_GOBILDA * GEAR_RATIO) / WHEEL_CIRCUMFERENCE;
    private static final double TPI = Math.PI * 2;

    public double startX = 0;
    public double startY = 0;
    public double startR = 0;

    public double Time ;
    double [] errors = new double[2];
    ElapsedTime time = new ElapsedTime();
    Boolean Dotimeout =true;

    double wantedAngle = 0;
    double NewAngle = 0;

    private final Pid xPid = new Pid(0.258, 0, 0, 0);
    private final Pid yPid = new Pid(0, 0, 0, 0);
    private final Pid rPid = new Pid(0.1001, 0, 0, 0);


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

        RFM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
      
        imu.initialize(parameters);

        reset();

        // x
        xPid.setTolerance(0);
        xPid.setIntegrationBounds(0,0);
        //y
        yPid.setIntegrationBounds(-0.2 ,0.2);
        yPid.setTolerance(10);
        //R
        rPid.setIntegrationBounds(0,0);
        rPid.setTolerance(Math.toRadians(0));



    }


    public void reset() {
        //reset the encoders that are used for Xr, Xl and Y
        LFM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RFM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LFM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        imu.resetYaw();


    }

    public void resetAngle( boolean h) {
        imu.resetYaw();
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
                robotHading_CWP =imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                robotHading_CCWP = -robotHading_CWP; //ccw is positive
            return NormalizeAngle(robotHading_CCWP); //normalize the angle to be between -pi and pi
    }

    public double getXlEncoder() {
        return LBM.getCurrentPosition();
    }

    public double getXrEncoder() {
        return RFM.getCurrentPosition();
    }

    public double getYEncoder() {
        return LFM.getCurrentPosition();
    }
    public double getFieldX() {
        return fieldX;
    }
    public double getFieldY() {
        return fieldY;
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
        Heading();
        double deltaX = deltaMiddlePos * cos(robotHading_CCWP) - deltaPerpPos * sin(robotHading_CCWP);
        double deltaY = deltaMiddlePos * sin(robotHading_CCWP) + deltaPerpPos * cos(robotHading_CCWP);

        //update the field position of the robot
        fieldX += ticksToMM(deltaX);
        fieldY += ticksToMM(deltaY);


        //save the encoder position for the next iteration of the loop
        prevLeftEncoderPos = leftEncoderPos;
        prevRightEncoderPos = rightEncoderPos;
        prevCenterEncoderPos = centerEncoderPos;

    }
    public double[]  fieldToRobotConvert(double deltaX ,double deltaY) {
        //convert the  field deltas to robot deltas
        double[] pos = new double[2];
        double robotDeltaX = deltaX * Math.cos(Heading()) - deltaY * Math.sin(Heading());
        double robotDeltaY = deltaX * Math.sin(Heading()) + deltaY * Math.cos(Heading());
        pos[0] = robotDeltaX;
        pos[1] = robotDeltaY;

        return pos;
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

    public void Drive(double X, double Y, double RX) {
        update();
        double heading = Heading();
        double rotX = 0;
        double rotY = 0;

        rotX = X * Math.cos(heading) - Y * Math.sin(heading);
        rotY = X * Math.sin(heading) + Y * Math.cos(heading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(RX), 1);

        double frontLeftPower = (rotY + rotX + RX) / denominator;
        double backLeftPower = (rotY - rotX + RX) / denominator;
        double frontRightPower = (rotY - rotX - RX) / denominator;
        double backRightPower = (rotY + rotX - RX) / denominator;

        LFM.setPower(frontLeftPower);
        LBM.setPower(backLeftPower);
        RFM.setPower(frontRightPower);
        RBM.setPower(backRightPower);

        opMode.telemetry.addData("Power lb", LBM.getPower());
        opMode.telemetry.addData("Power rb", RBM.getPower());
        opMode.telemetry.addData("Power rf", RFM.getPower());
        opMode.telemetry.addData("Power lf", LFM.getPower());
    }

    public boolean driveTo(double x, double y, double r, double timeOut) {
        double xPower = 0;
        double yPower = 0;
        double rPower = 0;
        double startTime = time.milliseconds();
        double[] curPos = new double[2];
        double[] dstPos = new double[2];

        //drive the robot to the position with the calculated power and the robot is field centric
        do{
            update();
            if (time.milliseconds()-startTime>timeOut){
                // stop the robot
                Drive(0,0,0);
                opMode.telemetry.clear();
                opMode.telemetry.addData("timeOut","timeOut");

                return false;
            }
            //calculate the error in the position
            curPos = fieldToRobotConvert(fieldX,fieldY);
            dstPos = fieldToRobotConvert(x,y);
            //calculate the power needed to get to the position
            xPower = xPid.calculate(curPos[0],dstPos[0]);
            yPower = yPid.calculate(curPos[1],dstPos[1]);
            rPower = rPid.calculate(Heading(),Math.toRadians(r));
            //limit the power to 0.7
            xPower = Range.clip(xPower, 0, 0);
            yPower = Range.clip(yPower, 0, 0);
            Drive(xPower,yPower, rPower);

        } while ((!xPid.atSetPoint() || !yPid.atSetPoint()|| !rPid.atSetPoint()) && opMode.opModeIsActive() && !opMode.isStopRequested());//if the robot is at the position (or the op mode is off) then stop the loop//stop the robot
        Drive(xPower, -yPower, rPower);
        //return true if the robot is at the position
        return true;
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


