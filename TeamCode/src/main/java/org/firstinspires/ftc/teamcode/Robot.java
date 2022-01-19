package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*

This code is used for driver controlled and Autonomous for the 2020-2021 season.

 */

@Config
public class Robot {

    //OpenCV ring sensing

    //these values are in percent
    public static double RingThreshold4 = 0.4; //any value above 0.5% will be considered 4 rings
    public static double RingThreshold1 = 0.2;  // any value above 0.15% will be considered 1 ring
    //else there are 0 rings
    public static int threshold = 119;      // the value that says what is a ring and what is not

    public static double shootingAngleHigh = 0.67;
    public static double shootingAnglePowershots = 0.77;

    public static  double MaxPower = 1800;
    public static double fireRate = 0.15;

    public static double reload = 0.3;
    public static double push = 0.45;

    public static double open = 0;
    public static double closed = 0.75;

    public static double up = 0.75;
    public static double down = 0.08;

    public Vector2d targetPosition = new Vector2d(72,36);// position of high goal

    public static double ShootingOffset = 0;//offset in DEGREES

    public static Pose2d poseStorage = new Pose2d();// Saves the position from autonomous to TeliOp

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;


    public DcMotorEx shooter;
    public DcMotor intake;
    public DcMotorEx wobbleGoal;
    public Servo pusher;
    public Servo grab;
    public Servo lift;
    public Servo ringPusher;


    ModernRoboticsI2cRangeSensor rangeLeft;
    ModernRoboticsI2cRangeSensor rangeRear;

    ColorSensor sensorColor;
    DistanceSensor sensorDistance;

    public RevBlinkinLedDriver blinkinLedDriver;

    public Robot(HardwareMap hardwareMap) {

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");
        wobbleGoal = hardwareMap.get(DcMotorEx.class, "wobbleGoal");
        pusher = hardwareMap.get(Servo.class, "pusher");//push rings
        grab = hardwareMap.get(Servo.class, "grab");//wobble goal
        lift = hardwareMap.get(Servo.class, "lift");//shooting angle
        ringPusher = hardwareMap.get(Servo.class, "ringPusher");//push over the stack of rings
        rangeLeft = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeRight");
        rangeRear = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeRear");

        // get a reference to the color sensor.
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);

        wobbleGoal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleGoal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleGoal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        blinkinLedDriver.setPattern(pattern);
    }

    public void setShooterSpeed(double s){
        shooter.setVelocity(s  * MaxPower);
    }

    public void setIntakeSpeed(double s){
        intake.setPower(s);
    }

    public void reloadLauncher(){
        pusher.setPosition(reload);
    }

    public void LaunchRings(){
        pusher.setPosition(push);
    }

    public void grabWobbleGoal(){
        grab.setPosition(closed);
    }

    public void releaseWobbleGoal(){
        grab.setPosition(open);
    }

    public void setLiftPosition(double position){
        lift.setPosition(position);
    }

    public void setWobbleGoalSpeed(double power) {
        wobbleGoal.setPower(power);
    }

    public double getShootingAngle(Pose2d robotPosition){
        return getShootingAngle(new Vector2d(robotPosition.getX(),robotPosition.getY()));
    }

    public double getShootingAngle(Vector2d robotPosition) {
        return Math.toRadians(ShootingOffset) + Math.atan(
                (robotPosition.getY() - targetPosition.getY()) /
                (robotPosition.getX() - targetPosition.getX())
        );
    }

    public void ringPusherUp(){
        ringPusher.setPosition(up);
    }

    public void ringPusherDown(){
        ringPusher.setPosition(down);
    }

    public double getRangeLeft(){
        return rangeLeft.getDistance(DistanceUnit.INCH);
    }

    public double getRangeRear(){
        return rangeRear.getDistance(DistanceUnit.INCH);
    }

    public void setTarget(String target){
        if(target.equals("left")){
            targetPosition = new Vector2d(72,30);
        }
        else if(target.equals("center")){
            targetPosition = new Vector2d(72,22);
        }
        else if(target.equals("right")){
            targetPosition = new Vector2d(72,15);
        }
        else{
            targetPosition = new Vector2d(72,42);
        }

    }

    public Pose2d getAngle(double Heading){//returns the robot's position on the field

        //TODO change this back if if dosen't work

        /*
        double rr = getRangeRear();
        double rl = getRangeLeft();
        double rlr = getRangeLeftRear();




        if(rl > rlr){
            double diff = rl - rlr;
            Heading = Math.atan(7.0 / diff);
        }
        else{
            if(rl == rlr){
                Heading = 0;
            }
            else {
                double diff = rlr - rl;
                Heading = Math.atan(7.0 / diff);
            }
        }

        double X = rr * Math.cos(Heading); //front-back
        X = -72 + (X + 9);
        double Y = rl * Math.cos(Heading); //left-right
        Y = 72 - (Y + 9);
        return new Pose2d(X,Y,Heading);
*/
        //TODO old code


        //Distance sensor localization
        double X = getRangeRear() * Math.cos(Heading); //front-back
        X = -72 + (X + 9);
        double Y = getRangeLeft() * Math.cos(Heading); //left-right
        Y = 72 - (Y + 9);
        return new Pose2d(X,Y,Heading);
    }

    public double getAlphaColor(){
        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);
        return sensorColor.alpha();
    }
}
