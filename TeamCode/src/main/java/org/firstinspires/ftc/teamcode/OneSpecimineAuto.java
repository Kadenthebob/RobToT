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
import org.firstinspires.ftc.teamcode.sections.Intake;
import org.firstinspires.ftc.teamcode.sections.Lifters;


@Autonomous(name = "1 Spec Auto", group = "Auto Testing")
public final class OneSpecimineAuto extends LinearOpMode {

    PathChain placeFirst, slideBlocks,grabSecond,placeSecond,grabThird,placeThird,grabFourth,placeFourth,hockeyFirst,hockeySecond,hockeyThird,hockeyGrab;
    Follower follower;

    Pose startPose = new Pose(9, 48, Math.toRadians(180));
    Pose hockey1 = new Pose(29, 33.5, Math.toRadians(-30));
    Pose hockey15 = new Pose(29, 33.5, Math.toRadians(-150));
    Pose hockey2 = new Pose(29, 23.5, Math.toRadians(-30));
    Pose hockey25 = new Pose(29, 23.5, Math.toRadians(-150));
    Pose hockey3 = new Pose(29, 13.5, Math.toRadians(-30));
    Pose grab1 = new Pose(9, 8.1, Math.toRadians(0));
    Pose grab2 = new Pose(9, 24, Math.toRadians(0));
    Pose grabCP = new Pose(28,24,Math.toRadians(0));
    Pose place1 = new Pose(36, 60, Math.toRadians(0));
    Pose place2 = new Pose(36, 61, Math.toRadians(0));
    Pose place3 = new Pose(36, 63, Math.toRadians(0));
    Pose place4 = new Pose(36, 65, Math.toRadians(0));
    Pose place5 = new Pose(36, 67, Math.toRadians(0));



    //All paths stored here
    public void buildPaths(){

        placeFirst = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(startPose),
                                new Point(place1)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        hockeyFirst = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(36.000, 59.000, Point.CARTESIAN),
                                new Point(29.000, 34.500, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-30))
                .build();

        hockeySecond = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(29.000, 34.500, Point.CARTESIAN),
                                new Point(29.000, 34.500, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(-150))
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(29.000, 34.500, Point.CARTESIAN),
                                new Point(29.000, 24.500, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-150), Math.toRadians(-30))
                .build();

        hockeyThird = follower.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(29.000, 24.500, Point.CARTESIAN),
                                new Point(29.000, 24.500, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(-150))
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(29.000, 24.500, Point.CARTESIAN),
                                new Point(29.000, 14.500, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-150), Math.toRadians(-30))
                .build();

        hockeyGrab = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(hockey3),
                                new Point(grab1)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(-180))
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
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(67.846, 23.769, Point.CARTESIAN),
                                new Point(17.769, 23.769, Point.CARTESIAN)
                        ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(17.769, 23.769, Point.CARTESIAN),
                                new Point(70.846, 25.385, Point.CARTESIAN),
                                new Point(67.615, 13.385, Point.CARTESIAN)
                        ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(67.615, 13.385, Point.CARTESIAN),
                                new Point(17.769, 13.385, Point.CARTESIAN)
                        ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Line 6
                        new BezierCurve(
                                new Point(17.769, 13.385, Point.CARTESIAN),
                                new Point(71.769, 15.462, Point.CARTESIAN),
                                new Point(67.846, 8.077, Point.CARTESIAN)
                        ))
                .setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(0))
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
                .setLinearHeadingInterpolation(Math.toRadians(0),Math.toRadians(180))
        .build();

        grabThird = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(place2),
                                new Point(grabCP),
                                new Point(grab2)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(0))
        .build();

        placeThird = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(grab2),
                                new Point(place3)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0),Math.toRadians(180))
        .build();

        grabFourth = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(place3),
                                new Point(grabCP),
                                new Point(grab2)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
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

        waitForStart();
        Actions.runBlocking(
           //Follower Class loop
            new ParallelAction(
                follower.Update(),
                //main sequential
                new SequentialAction(
                        intk.SetTrunkWall(),
                        intk.SetTwistPos(90),
                        //Aproach Pole
                        new ParallelAction(
                                follower.FollowPath(placeFirst),
                                new SequentialAction(
                                        lift.setVertLifterPos(2520,1)
                                )
                        ),
                        //1st Place on Pole
                        new SleepAction(2.5),
                        lift.setVertLifterPos(1500,1),
                        intk.IntakeOut(),
                        //Drag Samples to Human Player && Approach Human Player
//                        new ParallelAction(
//                                follower.FollowPath(hockeyFirst,true),
//                                new SequentialAction(
//                                        follower.waitForPoint(hockey1)
//                                )
//                        ),
//                        intk.IntakeIn(),
//                        lift.setVertLifterPos(0,1),
//                        lift.setVertLifterPos(680,1),
//                        intk.IntakeOff(),
//                        new ParallelAction(
//                                follower.FollowPath(hockeySecond,true),
//                                new SequentialAction(
//                                        follower.waitForPoint(hockey15),
//                                        intk.IntakeOut(),
//                                        follower.waitForPoint(hockey2)
//                                )
//                        ),
//                        intk.IntakeIn(),
//                        lift.setVertLifterPos(0,1),
//                        lift.setVertLifterPos(680,1),
//                        intk.IntakeOff(),
//                        new ParallelAction(
//                                follower.FollowPath(hockeyThird,true),
//                                new SequentialAction(
//                                        follower.waitForPoint(hockey25),
//                                        intk.IntakeOut(),
//                                        follower.waitForPoint(hockey3)
//                                )
//                        ),
                        new ParallelAction(
                                follower.FollowPath(slideBlocks,true),
                                new SequentialAction(
                                        lift.setVertLifterPos(0,1),
                                        intk.IntakeOff(),
                                        follower.waitForPoint(new Pose(67.846,8.1))
                                )
                        ),
                        follower.waitForPoint(grab1)

                )
            )
        );
    }

}
