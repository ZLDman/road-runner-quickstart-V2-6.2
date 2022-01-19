package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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


@Autonomous(name= "Blueautonomous 2020 OpenCV", group="Ultimate Goal Autonomous")

public class BlueAutonomous extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valtop = -1;
    private static int valbottom = -1;
    public static double percentRings = 0;
    public static int rings;


    private static float offsetX = 0f / 8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f / 8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] topPos = {3.35f / 8f + offsetX, 3f / 8f + offsetY};//0 = col, 1 = row;
    private static float[] bottomPos = {3.35f / 8f + offsetX, 4.5f / 8f + offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;
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

        Trajectory strafeOverTo1WobbleGoal = drive.trajectoryBuilder(new Pose2d(-63, 49.5, 0), false)
                .strafeTo(new Vector2d(-61,55))
                .build();


        Trajectory wobble1Rings0 = drive.trajectoryBuilder(strafeOverTo1WobbleGoal.end(), false)
                .lineTo(new Vector2d(0, 60))
                .build();

        Trajectory wobble1Rings1 = drive.trajectoryBuilder(strafeOverTo1WobbleGoal.end(), false)
                .splineTo(new Vector2d(-19,55),Math.toRadians(-10))
                .splineTo(new Vector2d(25, 36), Math.toRadians(0))
                .build();

        Trajectory wobble1Rings4 = drive.trajectoryBuilder(strafeOverTo1WobbleGoal.end(), false)
                        .splineTo(new Vector2d(-19,55),Math.toRadians(0))
                        .splineTo(new Vector2d(56, 60), Math.toRadians(0))
                        .build();

        Trajectory strafeOverTo2wobbleGoal = drive.trajectoryBuilder(new Pose2d(-65, 37, 0), false)
                .strafeTo(new Vector2d(-65,25))
                .build();

        Trajectory deliver2wobbleGoal = drive.trajectoryBuilder(strafeOverTo2wobbleGoal.end(), false)
                .lineTo(new Vector2d(15,2))
                .build();

        Trajectory overToShoot = drive.trajectoryBuilder(deliver2wobbleGoal.end(), true)
                .splineTo(new Vector2d(-3,31), Math.toRadians(180))
                .build();

        Trajectory Park = drive.trajectoryBuilder(overToShoot.end(), false)
                .splineTo(new Vector2d(10, 36), 0)
                .build();
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

            drive.setPoseEstimate(new Pose2d(-63, 49.5, 0));

            //Strafe over to 1st wobble goal
            Pose2d currentPose = drive.getPoseEstimate();

            drive.followTrajectory(strafeOverTo1WobbleGoal);

            sleep(500);


            //place the 1st wobble goal
            telemetry.addData("rings: ", rings);

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

            //backup
            currentPose = drive.getPoseEstimate();
            drive.followTrajectory(
                    drive.trajectoryBuilder(new Pose2d(currentPose.getX(), currentPose.getY(), currentPose.getHeading()), true)
                            .splineTo(new Vector2d(-19,55), Math.toRadians(180))
                            .splineTo(new Vector2d(-65,37), Math.toRadians(180))
                            .build()
            );

            //strafe to 2nd wobble goal
            drive.followTrajectory(strafeOverTo2wobbleGoal);

            //TODO place 2nd wobble goal (this code is for 0 rings need to add the rest)
            //forward
            drive.followTrajectory(deliver2wobbleGoal);

            //drive over to shoot
            drive.followTrajectory(overToShoot);

            /*
            drive.setPoseEstimate(new Pose2d(-63, 49.5, 0));

            Pose2d currentPose = drive.getPoseEstimate();
            drive.followTrajectory(
                    drive.trajectoryBuilder(currentPose, false)
                            .splineTo(new Vector2d(3, 56), Math.toRadians(45))
                            .build()
            );

            currentPose = drive.getPoseEstimate();
            drive.followTrajectory(
                    drive.trajectoryBuilder(currentPose, true)
                            .splineTo(new Vector2d(-5, 36), Math.toRadians(180))
                            .build()
            );
             */

            robot.setShooterSpeed(-0.8);

            robot.setLiftPosition(0.61);

            sleep(1000);

            robot.setTarget("high");
            Pose2d poseEstimate = drive.getPoseEstimate();
            drive.turn(Angle.normDelta(robot.getShootingAngle(poseEstimate) - poseEstimate.getHeading()));

            for (int i = 0; i < 3; i++) {
                robot.LaunchRings();
                sleep(250);
                robot.setShooterSpeed(-0.9);
                robot.reloadLauncher();
                sleep(250);
            }

            /*
            robot.setTarget("center");
            poseEstimate = drive.getPoseEstimate();
            drive.turn(Angle.normDelta(robot.getShootingAngle(poseEstimate) - poseEstimate.getHeading()));
            robot.LaunchRings();
            sleep(250);
            robot.reloadLauncher();
            sleep(250);

            robot.setTarget("right");
            poseEstimate = drive.getPoseEstimate();
            drive.turn(Angle.normDelta(robot.getShootingAngle(poseEstimate) - poseEstimate.getHeading()));
            robot.LaunchRings();
            sleep(250);
            robot.reloadLauncher();
            sleep(250);
             */


            robot.setShooterSpeed(0);

            drive.followTrajectory(Park);

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
            for(float i = 3f / 8f; i < 4.5f / 8f; i += 0.001) {
                double[] pixtop = thresholdMat.get((int) (input.rows() * i), (int) (input.cols() * 3.35f / 8f));//gets value at circle
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