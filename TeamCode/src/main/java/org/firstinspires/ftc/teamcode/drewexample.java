package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous

public class drewexample extends LinearOpMode {
    static int pos = 0;


    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-70, -40, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                //...
                .build();
        TrajectorySequence traj2a = drive.trajectorySequenceBuilder(traj1.end())
                //...
                .build();
        waitForStart();

               drive.followTrajectorySequence(traj1);
                sleep(999999);            }

    }
