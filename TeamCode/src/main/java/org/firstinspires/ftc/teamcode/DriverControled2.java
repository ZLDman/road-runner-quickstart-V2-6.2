package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
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
@TeleOp(name = "DriverControled2drivers",group = "drive")
public class DriverControled2 extends LinearOpMode {

    double LaunchTime;
    double LightsTime;
    double LightsStartTime = System.currentTimeMillis();
    double LaunchStartTime = 5;
    double position = Robot.shootingAngleHigh;

    double imuOffset = 0;

    @SuppressLint("Assert")
    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Robot robot = new Robot(hardwareMap);


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive.setPoseEstimate(Robot.poseStorage);//gets where the robot was at the end of autonomous

        Robot.shootingAngleHigh = 0.69;

        waitForStart();

        LightsStartTime = System.currentTimeMillis();

        robot.reloadLauncher();

        while (!isStopRequested()) {

            /*SHOOTING WHEELS*/
            robot.setShooterSpeed(-gamepad1.right_trigger);

            /*INTAKE*/
            if(Math.abs(gamepad2.left_stick_y) > 0.01){
                robot.setIntakeSpeed(-gamepad2.left_stick_y);
            }
            else if(gamepad2.left_bumper){
                robot.setIntakeSpeed(-gamepad2.left_trigger);
            }
            else {
                robot.setIntakeSpeed(gamepad2.left_trigger);
            }

            robot.setWobbleGoalSpeed(gamepad2.right_stick_y);

            robot.setWobbleGoalSpeed(gamepad2.right_stick_y);

            //ring pusher
            if(gamepad2.dpad_up) {
                robot.ringPusherUp();
            }
            else if(gamepad2.dpad_down){
                robot.ringPusherDown();
            }

            //grab wobble goal
            if(gamepad2.x) {
                robot.grabWobbleGoal();
            }
            else if(gamepad2.y){
                robot.releaseWobbleGoal();
            }


            /*PUSHING SERVO*/
            LaunchTime = (System.currentTimeMillis() - LaunchStartTime) / 1000;
            if(gamepad1.right_bumper && LaunchTime > Robot.fireRate){
                LaunchStartTime = System.currentTimeMillis();
            }
            if(LaunchTime < 0.125){
                robot.LaunchRings();
            }
            else if(LaunchTime < 0.25){
                robot.reloadLauncher();
            }


            /*Lift Servo*/
            robot.setLiftPosition(position);

            //shoot one ring
            if(gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_down || gamepad1.dpad_right){

                //selects what we are aiming for
                if (gamepad1.dpad_up) {
                    robot.setTarget("high");
                    position = Robot.shootingAngleHigh;
                } else if (gamepad1.dpad_down) {
                    robot.setTarget("center");
                    position = Robot.shootingAnglePowershots;
                } else if (gamepad1.dpad_left) {
                    robot.setTarget("left");
                    position = Robot.shootingAnglePowershots;
                } else if (gamepad1.dpad_right) {
                    position = Robot.shootingAnglePowershots;
                    robot.setTarget("right");
                }

                /*Lift Servo*/
                robot.setLiftPosition(position);

                shootOneRing: {
                    drive.update();
                    robot.setShooterSpeed(-1);

                    Pose2d poseEstimate = drive.getPoseEstimate();
                    drive.turn(Angle.normDelta(0 - poseEstimate.getHeading()));

                    if (gamepad1.a) break shootOneRing;

                    drive.setPoseEstimate(robot.getAngle(drive.getRawExternalHeading() - imuOffset));
                    drive.update();
                    poseEstimate = drive.getPoseEstimate();
                    drive.turn(Angle.normDelta(robot.getShootingAngle(poseEstimate) - poseEstimate.getHeading()));

                    if (gamepad1.a) break shootOneRing;

                    while (robot.shooter.getVelocity() > -Robot.MaxPower + 100 && opModeIsActive()) {
                        if (gamepad1.a) break shootOneRing;
                    }
                    robot.LaunchRings();
                    sleep((int) (Robot.fireRate * 500));
                }
                robot.reloadLauncher();
                robot.setShooterSpeed(0);
            }

            //Distance Sensor Localization
            drive.setPoseEstimate(robot.getAngle(drive.getRawExternalHeading() - imuOffset));

            //update position
            drive.update();

            //Reset heading
            if(gamepad1.x) {
                imuOffset = drive.getRawExternalHeading();
            }



            Pose2d poseEstimate = drive.getPoseEstimate();

            //Auto aim(high goal)
            /*if(gamepad1.b){
                //robot.setTarget("high");
                drive.turnAsync(Angle.normDelta(robot.getShootingAngle(poseEstimate) - poseEstimate.getHeading()));

                //robot.setLiftPosition(Robot.shootingAngleHigh);
                                                    //radians of high goal
                //drive.turnAsync(Angle.normDelta(0.165 - (drive.getRawExternalHeading() - imuOffset)));
            }*/

            //Auto aim 3 high goals
            if(gamepad1.left_bumper){
                autoAimHighGoal: {
                    robot.setTarget("high");
                    position = Robot.shootingAngleHigh;
                    drive.update();
                    robot.setLiftPosition(Robot.shootingAngleHigh);
                    robot.setShooterSpeed(-1);

                    //poseEstimate = drive.getPoseEstimate();
                    //drive.turn(Angle.normDelta(0 - poseEstimate.getHeading()));

                    if (gamepad1.a) break autoAimHighGoal;

                    drive.setPoseEstimate(robot.getAngle(drive.getRawExternalHeading() - imuOffset));
                    drive.update();
                    poseEstimate = drive.getPoseEstimate();
                    drive.turn(Angle.normDelta(robot.getShootingAngle(poseEstimate) - poseEstimate.getHeading()));

                    if (gamepad1.a) break autoAimHighGoal;

                    for (int i = 0; i < 3; i++) {
                        while (robot.shooter.getVelocity() > -Robot.MaxPower + 100 && opModeIsActive()) {
                            if (gamepad1.a) break autoAimHighGoal;
                        }
                        robot.LaunchRings();
                        sleep((int) (Robot.fireRate * 500));
                        robot.reloadLauncher();
                        if (gamepad1.a) break autoAimHighGoal;
                        sleep((int) (Robot.fireRate * 1000));
                    }
                }
                robot.reloadLauncher();

                robot.setShooterSpeed(0);
            }

            //quickly shoot all three power shots

            if(gamepad1.back) {
                autoAimPowerShots: {
                    robot.setLiftPosition(Robot.shootingAnglePowershots);
                    robot.setShooterSpeed(-1);

                    poseEstimate = drive.getPoseEstimate();
                    drive.turn(Angle.normDelta(0 - poseEstimate.getHeading()));

                    if (gamepad1.a) break autoAimPowerShots;
                    drive.setPoseEstimate(robot.getAngle(drive.getRawExternalHeading() - imuOffset));
                    drive.update();

                    //first ring left powershot
                    poseEstimate = drive.getPoseEstimate();
                    robot.setTarget("center");
                    drive.turn(Angle.normDelta(robot.getShootingAngle(poseEstimate) - poseEstimate.getHeading()));
                    while (robot.shooter.getVelocity() > -Robot.MaxPower + 100 && opModeIsActive()) {
                        if (gamepad1.a) break autoAimPowerShots;
                    }
                    robot.LaunchRings();
                    sleep(100);
                    robot.reloadLauncher();

                    if (gamepad1.a) break autoAimPowerShots;

                    //second ring center powershot
                    poseEstimate = drive.getPoseEstimate();
                    robot.setTarget("left");
                    drive.turn(Angle.normDelta(robot.getShootingAngle(poseEstimate) - poseEstimate.getHeading()));
                    while (robot.shooter.getVelocity() > -Robot.MaxPower + 100 && opModeIsActive()) {
                        if (gamepad1.a) break autoAimPowerShots;
                    }
                    robot.LaunchRings();
                    sleep(100);
                    robot.reloadLauncher();

                    if (gamepad1.a) break autoAimPowerShots;

                    //third ring right powershot
                    poseEstimate = drive.getPoseEstimate();
                    robot.setTarget("right");
                    drive.turn(Angle.normDelta(robot.getShootingAngle(poseEstimate) - poseEstimate.getHeading()));
                    while (robot.shooter.getVelocity() > -Robot.MaxPower + 100 && opModeIsActive()) {
                        if (gamepad1.a) break autoAimPowerShots;
                    }
                    robot.LaunchRings();
                    sleep(100);

                }
                robot.reloadLauncher();

                robot.setShooterSpeed(0);

                robot.setTarget("high");
                position = Robot.shootingAngleHigh;
            }


            //speed control
            //Maps the speed between 1/2 and 1
            double speed = 1;


            if(!drive.isBusy()) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y * speed,
                                -gamepad1.left_stick_x * speed,
                                -gamepad1.right_stick_x * speed * 0.5
                        )
                );
            }

            //telemetry.addData("Wobble Goal", robot.wobbleGoal.getCurrentPosition());
            //telemetry.addData("position", robot.wobbleGoal.getCurrentPosition());
            //telemetry.addData("position2", robot.wobbleGoalHoldPosition);

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", drive.getRawExternalHeading());

            //telemetry.addData("target", robot.targetPosition);
            //telemetry.addData("imu heading with offset(deg)", Heading);
            //telemetry.addData("offset", imuOffset);
            telemetry.update();



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

        }
    }
}

