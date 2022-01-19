package org.firstinspires.ftc.teamcode.openCV;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;


@Autonomous(name= "opencvSkystoneDetectorWebcam", group="Sky autonomous")
//@Disabled//comment out this line before using
public class opencvSkystoneDetectorWebcam extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valtop = -1;
    private static int valbottom = -1;
    public static int total = 0;
    public static boolean onLine = false;
    public static boolean turn = false;

    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] topPos = {4f/8f+offsetX, 1.75f/8f+offsetY};//0 = col, 1 = row;
    private static float[] bottomPos = {4f/8f+offsetX, 3.25f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;
    ///OpenCvCamera phoneCam;
    OpenCvCamera webcam;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Robot robot = new Robot(hardwareMap);

        int rings = 0;
        //robot.LiftBreak();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        //P.S. if you're using the latest version of easyopencv, you might need to change the next line to the following:
        //phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        //webcam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);//remove this

        webcam.openCameraDevice();//open camera
        webcam.setPipeline(new StageSwitchingPipeline());//different stages
        webcam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.

        while (!isStarted()) {
            rings = total;
            telemetry.addData("total", rings);
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

        runtime.reset();

        while(!onLine && opModeIsActive()) {
            drive.setMotorPowers(0.5,0.5,0.6,0.6);
        }
        while(turn && opModeIsActive()){
            drive.setMotorPowers(0.3,0.3,-0.3,-0.3);
        }
        drive.setMotorPowers(0,0,0,0);
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

        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();

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
            //Imgproc.threshold(yCbCrChan2Mat, thresholdMat, Robot.theshold2, 255, Imgproc.THRESH_BINARY_INV);

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

            double percent = 0;
            int total = 0;
            for(float i = 4f/8f; i < 7f/8f; i += 0.001) {
                double[] pixtop = thresholdMat.get((int) (input.rows() * 6f/8f), (int) (input.cols() * i));//gets value at circle
                valtop = (int) pixtop[0];
                percent += (double) valtop / 255;
                total ++;
            }
            percent /= total;
            onLine = percent < 0.5;


            total = 0;
            for(float i = 1.25f/8f; i < 2.5f/8f; i += 0.001) {
                double[] pixtop = thresholdMat.get((int) (input.rows() * 0.5f/8f), (int) (input.cols() * i));//gets value at circle
                valtop = (int) pixtop[0];
                percent += (double) valtop / 255;
                total ++;
            }
            percent /= total;

            turn = percent > 0.75;

            Imgproc.putText(all, "%" + percent,new Point((int)(input.cols()* 1f / 8f), (int)(input.rows()* 5f / 8f)),1,5,new Scalar( 255, 0, 0 ),10);





            Imgproc.putText(all, "%" + percent,new Point((int)(input.cols()* 1f / 8f), (int)(input.rows()* 7f / 8f)),1,5,new Scalar( 255, 0, 0 ),10);




            //create three points
            Point pointtop = new Point((int)(input.cols()* 2.5f/8f), (int)(input.rows()* 0.5f/8f));
            Point pointbottom = new Point((int)(input.cols()* 1.25f/8f), (int)(input.rows()* 0.5f/8f));

            //draw circles on those points
            Imgproc.circle(all, pointtop,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointbottom,5, new Scalar( 255, 0, 0 ),1 );//draws circle


            switch (stageToRenderToViewport)
            {
                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case detection:
                {
                    return all;
                }

                case RAW_IMAGE:
                {
                    return input;
                }

                default:
                {
                    return input;
                }
            }
        }

    }
}