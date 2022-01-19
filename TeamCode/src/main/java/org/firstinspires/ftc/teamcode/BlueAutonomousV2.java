package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;


@Autonomous(name= "V2 Blueautonomous 2020 OpenCV ", group="Ultimate Goal Autonomous")

public class BlueAutonomousV2 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valtop = -1;
    private static int valbottom = -1;
    public static double percentRings = 0;
    public static int rings;


    private static float offsetX = 0f / 8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f / 8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] topPos = {6.5f / 8f + offsetX, 4.5f / 8f + offsetY};//0 = col, 1 = row;
    private static float[] bottomPos = {6.5f / 8f + offsetX, 6.5f / 8f + offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;


    double LaunchTime;
    double LaunchStartTime = 5;
    ///OpenCvCamera phoneCam;
    OpenCvCamera webcam;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Robot robot = new Robot(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        //P.S. if you're using the latest version of easyopencv, you might need to change the next line to the following:
        //phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);//remove this

        webcam.openCameraDevice();//open camera
        webcam.setPipeline(new StageSwitchingPipeline());//different stages
        webcam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC

        robot.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);

        drive.setPoseEstimate(new Pose2d(-63, 49.5, 0));
        
        Trajectory wobble1Rings0 = drive.trajectoryBuilder(new Pose2d(-63, 49.5, 0), false)
                .lineTo(new Vector2d(0, 60))
                .build();

        Trajectory wobble1Rings1 = drive.trajectoryBuilder(new Pose2d(-63, 49.5, 0), false)
                .splineTo(new Vector2d(-19,55),Math.toRadians(-10))
                .splineTo(new Vector2d(25, 35), Math.toRadians(0))
                .build();

        Trajectory wobble1Rings4 = drive.trajectoryBuilder(new Pose2d(-63, 49.5, 0), false)
                        .splineTo(new Vector2d(-19,55),Math.toRadians(0))
                        .splineTo(new Vector2d(47, 60), Math.toRadians(0))
                        .build();

        Trajectory wobble2Rings0 = drive.trajectoryBuilder(new Pose2d(-38,31, Math.toRadians(180)), false)
                .splineTo(new Vector2d(5, 56),Math.toRadians(90))
                .build();

        Trajectory wobble2Rings1 = drive.trajectoryBuilder(new Pose2d(-38,31, Math.toRadians(180)), true)
                .strafeTo(new Vector2d(-19,28))
                .build();

        Trajectory wobble2Rings12 = drive.trajectoryBuilder(new Pose2d(-19,31, Math.toRadians(180)), true)
                .strafeTo(new Vector2d(20, 38))
                .build();

        Trajectory wobble2Rings4 = drive.trajectoryBuilder(new Pose2d(-38,31, Math.toRadians(180)), true)
                .strafeTo(new Vector2d(-19,31))
                .build();

        Trajectory wobble2Rings42 = drive.trajectoryBuilder(new Pose2d(-19,31, Math.toRadians(180)), true)
                .strafeTo(new Vector2d(42, 60))
                .build();

        //detect rings
        int rings2 = 0;
        while (!isStarted()) {
            rings2 = rings;
            telemetry.addData("rings: ", rings);
            telemetry.addData("percentRings ", percentRings);
            telemetry.addData("Values", valtop+"   "+valbottom);
            telemetry.addData("Height", rows);
            telemetry.addData("Width", cols);
            //telemetry.update();
            sleep(100);
            //call movement functions
//            strafe(0.4, 200);
//            moveDistance(0.4, 700);
            telemetry.update();
        }
        if(!isStopRequested()) {

            webcam.stopStreaming();

            //set starting position
            drive.setPoseEstimate(new Pose2d(-63, 49.5, 0));

            robot.setIntakeSpeed(1);
            sleep(15);
            robot.setIntakeSpeed(0);

            //deliver first wobble goal
            if(rings2 == 0) {
                telemetry.addData("rings = ", "0");
                telemetry.update();
                drive.followTrajectory(wobble1Rings0);
            }
            else if(rings2 == 1){
                telemetry.addData("rings = ", "1");
                telemetry.update();
                drive.followTrajectory(wobble1Rings1);
            }
            else{
                telemetry.addData("rings = ", "4");
                telemetry.update();
                drive.followTrajectory(wobble1Rings4);
            }

            //backup to the pile of rings
            Pose2d currentPose = drive.getPoseEstimate();
            drive.followTrajectory(
                    drive.trajectoryBuilder(new Pose2d(currentPose.getX(), currentPose.getY(), currentPose.getHeading()), true)
                            .splineTo(new Vector2d(-36,54), Math.toRadians(180))
                            .build()
            );
            currentPose = drive.getPoseEstimate();
            drive.followTrajectory(
                    drive.trajectoryBuilder(new Pose2d(currentPose.getX(), currentPose.getY(), currentPose.getHeading()), true)
                            .splineToLinearHeading(new Pose2d(-45, 38, Math.toRadians(0)), Math.toRadians(90))
                            .build()
            );

            //line up to shoot
            robot.setShooterSpeed(-0.9);
            robot.setTarget("high");
            currentPose = drive.getPoseEstimate();
            drive.turn(Angle.normDelta(robot.getShootingAngle(currentPose) - currentPose.getHeading()));
            robot.ringPusherDown();
            //shoot first 3 rings
            for (int i = 0; i < 4; i++) {
                robot.LaunchRings();
                sleep(125);
                robot.setShooterSpeed(-0.9);
                robot.reloadLauncher();
                sleep(125);
            }

            // pick up the other ring if they are there
            if(rings2 >= 1) {
                currentPose = drive.getPoseEstimate();
                while (currentPose.getX() < -18 && opModeIsActive()) {
                    drive.update();
                    currentPose = drive.getPoseEstimate();
                    robot.setIntakeSpeed(1);
                    robot.setShooterSpeed(-0.9);
                    double correction = Angle.normDelta(robot.getShootingAngle(currentPose) - currentPose.getHeading());
                    drive.setMotorPowers(0.2 - correction, 0.2 - correction, 0.2 + correction, 0.2 + correction);

                    /*PUSHING SERVO*/
                    LaunchTime = (System.currentTimeMillis() - LaunchStartTime) / 1000;
                    if (LaunchTime > 0.25) LaunchStartTime = System.currentTimeMillis();
                    if (LaunchTime < 0.125) robot.LaunchRings();
                    else if (LaunchTime < 0.25) robot.reloadLauncher();
                }
            }
            else{
                currentPose = drive.getPoseEstimate();
                while (currentPose.getX() < -18 && opModeIsActive()) {
                    drive.update();
                    robot.setIntakeSpeed(0);
                    robot.setShooterSpeed(0);
                    currentPose = drive.getPoseEstimate();
                    double correction = Angle.normDelta(robot.getShootingAngle(currentPose) - currentPose.getHeading());
                    drive.setMotorPowers(0.5 - correction, 0.5 - correction, 0.5 + correction, 0.5 + correction);
                }
            }

            //stop intake/shooting wheel
            drive.setMotorPowers(0,0,0,0);
            robot.setIntakeSpeed(0);
            robot.setShooterSpeed(0);
            robot.ringPusherUp();

            //raise and open wobble goal
            robot.setWobbleGoalSpeed(-0.75);
            robot.grabWobbleGoal();
            sleep(2000);
            /*while(robot.wobbleGoal.getCurrentPosition() > -300){
                telemetry.addData("position" , robot.wobbleGoal.getCurrentPosition());
                telemetry.update();
            }*/
            robot.setWobbleGoalSpeed(0);

            //slower speed
            DriveConstants.MAX_VEL = 15;
            DriveConstants.MAX_ANG_VEL = 0.5;

            //drive over to 2nd wobble goal
            currentPose = drive.getPoseEstimate();
            drive.followTrajectory(
                    drive.trajectoryBuilder(new Pose2d(currentPose.getX(), currentPose.getY(), currentPose.getHeading()), true)
                            .splineTo(new Vector2d(-30,31), Math.toRadians(180))
                            .build()
            );

            currentPose = drive.getPoseEstimate();
            while (currentPose.getX() > -43 && opModeIsActive()) {
                drive.update();
                currentPose = drive.getPoseEstimate();
                double correction = Angle.normDelta(robot.getShootingAngle(currentPose) - currentPose.getHeading());
                drive.setMotorPowers(-0.3 - correction, -0.3 - correction, -0.3 + correction, -0.3 + correction);
            }

            drive.setMotorPowers(0,0,0,0);

            //grab 2nd wobble goal

            robot.setWobbleGoalSpeed(0.3);
            sleep(250);
            robot.releaseWobbleGoal();
            sleep(250);
            robot.setWobbleGoalSpeed(0);


            //set speed back to normal
            DriveConstants.MAX_VEL = 50;
            DriveConstants.MAX_ANG_VEL = Math.toRadians(90);

            //deliver second wobble goal
            if(rings2 == 0) {

                currentPose = drive.getPoseEstimate();
                drive.followTrajectory(
                        drive.trajectoryBuilder(new Pose2d(currentPose.getX(), currentPose.getY(), currentPose.getHeading()), true)
                                .forward(52)
                                .build()
                );

                drive.turn(Math.toRadians(-100));

                currentPose = drive.getPoseEstimate();
                drive.followTrajectory(
                        drive.trajectoryBuilder(new Pose2d(currentPose.getX(), currentPose.getY(), currentPose.getHeading()), true)
                                .back(12)
                                .build()
                );
            }
            else if(rings2 == 1){
                //turn 180
                drive.turn(Math.toRadians(180));
                drive.followTrajectory(wobble2Rings1);
                drive.followTrajectory(wobble2Rings12);
            }
            else{
                //turn 180
                drive.turn(Math.toRadians(180));
                drive.followTrajectory(wobble2Rings4);
                drive.followTrajectory(wobble2Rings42);
            }

            //release wobble goal
            robot.grabWobbleGoal();

            drive.update();
            currentPose = drive.getPoseEstimate();
            //park
            if(rings2 == 4){
                currentPose = drive.getPoseEstimate();
                drive.followTrajectory(
                        drive.trajectoryBuilder(new Pose2d(currentPose.getX(), currentPose.getY(), currentPose.getHeading()), true)
                            .forward(40)
                            .build()
                );
            }
            else{
                drive.followTrajectory(
                        drive.trajectoryBuilder(new Pose2d(currentPose.getX(), currentPose.getY(), currentPose.getHeading()), true)
                                .forward(3)
                                .build()
                );
            }


            Robot.poseStorage = drive.getPoseEstimate();

        }
    }
    //detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat output = new Mat();

        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage
        {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private OldBlueAutonomous.StageSwitchingPipeline.Stage stageToRenderToViewport = OldBlueAutonomous.StageSwitchingPipeline.Stage.detection;
        private OldBlueAutonomous.StageSwitchingPipeline.Stage[] stages = OldBlueAutonomous.StageSwitchingPipeline.Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey



            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            //Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores


            //takes the values from
            //Imgproc.threshold(output, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, Robot.threshold, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours



            //RGB values
            //double[] pixMid = input.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            //valLeft = (int)pixMid[0];
            //valMid = (int)pixMid[1];
            //valRight = (int)pixMid[2];

            //get values from frame

            /*
                private static float[] topPos = {3.35f / 8f + offsetX, 2.6f / 8f + offsetY};//0 = col, 1 = row;
                private static float[] bottomPos = {3.35f / 8f + offsetX, 4.1f / 8f + offsetY};
             */
            double percent = 0;
            int total = 0;
            for(float i = 4.5f / 8f; i < 6.5f / 8f; i += 0.001) {
                double[] pixtop = thresholdMat.get((int) (input.rows() * i), (int) (input.cols() * 6.5f / 8f));//gets value at circle
                valtop = (int) pixtop[0];
                percent += (double) valtop / 255;
                total ++;
            }
            percent /= total;

            percentRings = percent;

            if(percentRings > Robot.RingThreshold4){
                rings = 4;
            }
            else if(percentRings > Robot.RingThreshold1){
                rings = 1;
            }
            else{
                rings = 0;
            }

            //create three points
            Point pointtop = new Point((int)(input.cols()* topPos[0]), (int)(input.rows()* topPos[1]));
            Point pointbottom = new Point((int)(input.cols()* bottomPos[0]), (int)(input.rows()* bottomPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointtop,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointbottom,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //if()
            Imgproc.putText(all, "%" + percentRings,new Point((int)(input.cols()* 0.25f / 8f), (int)(input.rows()* 1.5f / 8f)),1,5,new Scalar( 255, 0, 0 ),10);
            Imgproc.putText(all, "Rings: " + rings,new Point((int)(input.cols()* 0.25f / 8f), (int)(input.rows()* 5.32f / 8f)),1,5,new Scalar( 255, 0, 0 ),10);


            switch (stageToRenderToViewport)
            {
                case THRESHOLD:
                {
                    return input;
                }

                case detection:
                {
                    return all;
                }

                default:
                {
                    return thresholdMat;
                }
            }
        }

    }
}