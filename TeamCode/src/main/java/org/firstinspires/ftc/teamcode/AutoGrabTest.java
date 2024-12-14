package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.sections.Camera;
import org.firstinspires.ftc.teamcode.sections.Intake;
import org.firstinspires.ftc.teamcode.sections.Lifters;


@Autonomous(name = "Auto Grab", group = "Auto Testing")
public final class AutoGrabTest extends LinearOpMode {

    PathChain placeFirst, slideBlocks,grabSecond,placeSecond,grabThird,placeThird,grabFourth,placeFourth,camGrab;
    Follower follower;

    Pose startPose = new Pose(0, 0, Math.toRadians(0));
    Pose grab1 = new Pose(9, 8.1, Math.toRadians(0));
    Pose grab2 = new Pose(9, 24, Math.toRadians(0));
    Pose grabCP = new Pose(28,24,Math.toRadians(0));
    Pose place1 = new Pose(36, 59, Math.toRadians(0));
    Pose place2 = new Pose(36, 61, Math.toRadians(0));
    Pose place3 = new Pose(36, 63, Math.toRadians(0));
    Pose place4 = new Pose(36, 65, Math.toRadians(0));
    Pose place5 = new Pose(36, 67, Math.toRadians(0));



    //All paths stored here
    public void buildPaths(){
//        camGrab = follower.pathBuilder()
//                .addPath(
//                        // Line 1
//                        new BezierLine(
//                                new Point(startPose),
//                                new Point(15,0)
//                        )
//
//
//                )

//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//                .build();

        placeFirst = follower.pathBuilder()
                    .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(startPose),
                                new Point(place1)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        slideBlocks = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(place1),
                                new Point(22.154, 20.077, Point.CARTESIAN),
                                new Point(70.385, 46.154, Point.CARTESIAN),
                                new Point(67.846, 23.769, Point.CARTESIAN)
                        ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(67.846, 23.769, Point.CARTESIAN),
                                new Point(17.769, 23.769, Point.CARTESIAN)
                        ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(17.769, 23.769, Point.CARTESIAN),
                                new Point(70.846, 25.385, Point.CARTESIAN),
                                new Point(67.615, 13.385, Point.CARTESIAN)
                        ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(67.615, 13.385, Point.CARTESIAN),
                                new Point(17.769, 13.385, Point.CARTESIAN)
                        ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 6
                        new BezierCurve(
                                new Point(17.769, 13.385, Point.CARTESIAN),
                                new Point(71.769, 15.462, Point.CARTESIAN),
                                new Point(67.846, 8.077, Point.CARTESIAN)
                        ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(67.846, 8.1, Point.CARTESIAN),
                                new Point(grab1)
                        ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        placeSecond = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(grab1),
                                new Point(place2)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
        .build();

        grabThird = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(place2),
                                new Point(grabCP),
                                new Point(grab2)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
        .build();

        placeThird = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(grab2),
                                new Point(place3)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
        .build();

        grabFourth = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(place3),
                                new Point(grabCP),
                                new Point(grab2)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
        .build();

        placeFourth = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(grab2),
                                new Point(place4)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
        .build();

    }

    @Override
    public void runOpMode() throws InterruptedException {


        follower = new Follower(hardwareMap);
        follower.resetIMU();
        buildPaths();
        follower.setStartingPose(startPose);

        Lifters lift = new Lifters(hardwareMap);
        Intake intk = new Intake(hardwareMap);
        Camera cam = new Camera(hardwareMap,"red");

        waitForStart();
        Actions.runBlocking(
           //Follower Class loop
            new ParallelAction(
                follower.Update(),
                //main sequential
                new SequentialAction(
                        follower.AutoGrabWallLoopTraj(cam,15)

                )
            )
        );
    }

}
