package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Config
@TeleOp(name = "DriverControled1driver",group = "drive")
public class DriverControled extends LinearOpMode {

    double LaunchTime;
    double LightsTime;
    double LightsStartTime = System.currentTimeMillis();
    double LaunchStartTime = 5;
    double position = 0.61;

    Pose2d TargetPose2D = new Pose2d();


    @SuppressLint("Assert")
    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Robot robot = new Robot(hardwareMap);


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive.setPoseEstimate(Robot.poseStorage);//gets where the robot was at the end of autonomous

        waitForStart();

        LightsStartTime = System.currentTimeMillis();


        while (!isStopRequested()) {

            /*SHOOTING WHEELS*/
            robot.setShooterSpeed(-gamepad1.right_trigger);



            /*INTAKE*/
            if(gamepad1.left_bumper){
                robot.setIntakeSpeed(-gamepad1.left_trigger);
            }
            else {
                robot.setIntakeSpeed(gamepad1.left_trigger);
            }

            /*PUSHING SERVO*/
            LaunchTime = (System.currentTimeMillis() - LaunchStartTime) / 1000;
            if(gamepad1.right_bumper && LaunchTime > Robot.fireRate){
                LaunchStartTime = System.currentTimeMillis();
            }
            if(LaunchTime < (Robot.fireRate/2)){
                robot.LaunchRings();
            }
            else if(LaunchTime < Robot.fireRate){
                robot.reloadLauncher();
            }


            /*Lift Servo*/
            if(gamepad1.dpad_up && position < 0.9){
                position += 0.01;
            }
            else if(gamepad1.dpad_down && position > 0.3){
                position -= 0.01;
            }
            else if(gamepad1.dpad_left){
                position = 0.68;
            }
            else if(gamepad1.dpad_right){
                position = 0.76;
            }

            robot.setLiftPosition(position);

            /*WOBBLE GOAL*/
            robot.setWobbleGoalSpeed(gamepad1.right_stick_y);

            double Heading = drive.getRawExternalHeading();
            double X = robot.getRangeRear() * Math.cos(Heading); //front-back
            X = -72 + (X + 9);
            double Y = 0;

            Y = robot.getRangeLeft() * Math.cos(Heading); //left-right
            Y = 72 - (Y + 9);

            drive.setPoseEstimate(new Pose2d(X,Y,Heading));


            //grab wobble goal
            if(gamepad1.x) {
                robot.grabWobbleGoal();
            }
            else if(gamepad1.y){
                robot.releaseWobbleGoal();
            }

            /*DRIVING WHEELS*/
            Pose2d poseEstimate = drive.getPoseEstimate();

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y / 1.5,
                            -gamepad1.left_stick_x / 1.5,
                            -gamepad1.right_stick_x / 2
                    )
            );

            drive.update();
            telemetry.addData("right", robot.getRangeLeft());
            telemetry.addData("rear", robot.getRangeRear());
            telemetry.addData("shooting speed", robot.shooter.getVelocity());
            telemetry.addData("position", position);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();


/*
            LightsTime = (System.currentTimeMillis() - LightsStartTime) / 1000;

            if(LightsTime < 60){
                robot.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            }
            else if(LightsTime < 75){
                robot.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
            }
            else if(LightsTime < 90){
                robot.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
            }
            else if(LightsTime < 105){
                robot.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            }
            else if(LightsTime < 110){
                robot.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_RED);
            }
            else if(LightsTime < 115){
                robot.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
            }
            else if(LightsTime < 120){
                robot.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
            }
            else{
               robot.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
            }

 */
        }

    }
}