package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Robot robot = new Robot(hardwareMap);

        waitForStart();

        drive.setPoseEstimate(new Pose2d(-63, 49.5, 0));

        if (isStopRequested()) return;

        Trajectory traj = drive.trajectoryBuilder(new Pose2d(-63, 49.5, 0))
                .splineTo(new Vector2d(-3, 40),0)


                .build();

        drive.followTrajectory(traj);

        robot.setShooterSpeed(-1);

        robot.setLiftPosition(Robot.shootingAngleHigh + 0.02);

        sleep(2000);

        for (int x = 0; x < 3; x++) {
            robot.LaunchRings();
            sleep(250);
            robot.reloadLauncher();
            sleep(250);
        }

        drive.followTrajectory(
                drive.trajectoryBuilder(traj.end(), true)
                        .forward(10)
                        .build()
        );
    }
}
