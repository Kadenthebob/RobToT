package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.sections.Intake;
import org.firstinspires.ftc.teamcode.sections.Lifters;


@Autonomous(name = "HP Auto", group = "Auto Testing")
public final class HumanPlayerAuto extends LinearOpMode {

    PathChain placeFirst,hockeyFirstPlace,grabFirst,hockeySecondPlace,placeSecond,grabSecond,grabThird,hockeyFirst,hockeySecond,hockeyThird,grabFourth,placeFifth,placeThird,placeFourth;
    Follower follower;

    Pose startPose = new Pose(7.59375, 48, Math.toRadians(0));
    Pose hockey1 = new Pose(30.75, 35, Math.toRadians(-45));
    Pose hockey15 = new Pose(26, 35, Math.toRadians(-150));
    Pose hockey2 = new Pose(30.75, 26.5, Math.toRadians(-45));
    Pose hockey25 = new Pose(28, 26.5, Math.toRadians(-150));
    Pose hockey3 = new Pose(63, 9.5, Math.toRadians(0));
    Pose grab1 = new Pose(7.6,9.5, Math.toRadians(0));
    Pose grab = new Pose(15,30, Math.toRadians(0));
    Pose forward = new Pose(7.6,30, Math.toRadians(0));
    Pose place1 = new Pose(42, 69, Math.toRadians(0));
    Pose place2 = new Pose(40, 66, Math.toRadians(0));
    Pose place3 = new Pose(40, 64, Math.toRadians(0));
    Pose place4 = new Pose(40, 62, Math.toRadians(0));
    Pose place5 = new Pose(40, 61, Math.toRadians(0));



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
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        hockeyFirst = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(place1),
                                new Point(20.000,70.000,Point.CARTESIAN),
                                new Point(hockey1)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .build();

        hockeyFirstPlace = follower.pathBuilder()
                .addPath(
                        new BezierPoint(
                            new Point(hockey15)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-150))
                .build();

        hockeySecond = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(hockey15),
                                new Point(hockey2)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-150), Math.toRadians(-45))
                .build();
        hockeySecondPlace= follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(hockey2),
                                new Point(hockey25)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-150))
                .build();

        hockeyThird = follower.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(hockey25),
                                new Point(hockey3)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-150), Math.toRadians(0))
                .build();

        grabFirst = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(hockey3),
                                new Point(grab1)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        placeSecond = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(grab1),
                                new Point(35,75, Point.CARTESIAN),
                                new Point(place2)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        grabSecond = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(place2),
                                new Point(grab)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(grab),
                                new Point(forward)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        placeThird = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(grab),
                                new Point(35,75, Point.CARTESIAN),
                                new Point(place3)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        grabThird = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(place3),
                                new Point(grab)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(grab),
                                new Point(forward)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        placeFourth = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(grab),
                                new Point(35,75, Point.CARTESIAN),
                                new Point(place4)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        grabFourth = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(place4),
                                new Point(grab)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(grab),
                                new Point(forward)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        placeFifth= follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(grab),
                                new Point(35,75, Point.CARTESIAN),
                                new Point(place5)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
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
                        //Aproach Pole
                        new ParallelAction(
                                follower.FollowPath(placeFirst),
                                intk.SetTwistPos(90),
                                intk.SetElbowPos(115),
                                intk.SetTrunkPos(95),
                                intk.SetClawClose(),
                                new SequentialAction(
                                        lift.setVertLifterPos(1200,1),
                                        follower.waitForPoint(place1)
                                )
                        ),
                        lift.setVertLifterPos(1700,1),
                        intk.SetClawOpen(),

                        new ParallelAction(
                                follower.FollowPath(hockeyFirst,true),
                                new SequentialAction(
                                        new SleepAction(1),
                                        lift.SetHorLifterPos(79.5), //.53
                                        intk.SetTrunkPit(),
                                        intk.SetClawOpen(),
                                        intk.SetTwistPos(45)//.0815

                                ),
                                new SequentialAction(
                                        new SleepAction(1.5),
                                        lift.setVertLifterPos(100,1),
                                        follower.waitForPose(hockey1)

                                )
                        ),
                        new SleepAction(.3),
                        intk.SetClawClose(),
                        new SleepAction(.3),
                        new ParallelAction(
                                follower.FollowPath(hockeyFirstPlace,true),
                                new SequentialAction(
                                        follower.waitForPose(hockey15)

                                )
                        ),
                        intk.SetClawOpen(),
                        new ParallelAction(
                                follower.FollowPath(hockeySecond,true),
                                new SequentialAction(
                                        follower.waitForPose(hockey2)

                                )
                        ),
                        lift.setVertLifterPos(0,1),
                        new SleepAction(.3),
                        intk.SetClawClose(),
                        new SleepAction(.3),
                        new ParallelAction(
                                follower.FollowPath(hockeySecondPlace,true),
                                new SequentialAction(
                                        lift.setVertLifterPos(240,1),
                                        follower.waitForPose(hockey25)

                                )
                        ),
                        intk.SetClawOpen(),
                        new ParallelAction(
                                follower.FollowPath(hockeyThird,true),
                                new SequentialAction(
                                        lift.SetHorLifterPos(0), //.53
                                        intk.SetTrunkWall()
                                ),
                                new SequentialAction(
                                        follower.waitForPose(hockey3)

                                )
                        ),
                        new ParallelAction(
                                follower.FollowPath(grabFirst,true),
                                intk.SetElbowPos(300),
                                intk.SetTwistPos(90),
                                intk.SetTrunkPos(60),
                                new SequentialAction(
                                        follower.waitForPose(grab1)
                                )
                        ),
                        intk.SetClawClose(),
                        new SleepAction(.3),
                        new ParallelAction(
                                follower.FollowPath(placeSecond,true),
                                intk.SetTwistPos(90),
                                intk.SetElbowPos(115),
                                intk.SetTrunkPos(95),
                                new SequentialAction(
                                        lift.setVertLifterPos(1150,1),
                                        follower.waitForPose(place2)
                                )
                        ),
                        lift.setVertLifterPos(1700,1),
                        intk.SetClawOpen(),
                        new ParallelAction(
                                follower.FollowPath(grabSecond,true),
                                intk.SetElbowPos(300),
                                intk.SetTwistPos(90),
                                intk.SetTrunkPos(60),
                                new SequentialAction(
                                        lift.setVertLifterPos(240,1),
                                        follower.waitForPose(grab)
                                )
                        ),
                        intk.SetClawClose(),
                        new SleepAction(.3),
                        new ParallelAction(
                                follower.FollowPath(placeThird,true),
                                intk.SetTwistPos(90),
                                intk.SetElbowPos(115),
                                intk.SetTrunkPos(95),
                                new SequentialAction(
                                        lift.setVertLifterPos(1150,1),
                                        follower.waitForPose(place3)
                                )
                        ),
                        lift.setVertLifterPos(1700,1),
                        intk.SetClawOpen(),
                        new ParallelAction(
                                follower.FollowPath(grabThird,true),
                                intk.SetElbowPos(300),
                                intk.SetTwistPos(90),
                                intk.SetTrunkPos(60),
                                new SequentialAction(
                                        lift.setVertLifterPos(240,1),
                                        follower.waitForPose(grab)
                                )
                        ),
                        intk.SetClawClose(),
                        new SleepAction(.3),
                        new ParallelAction(
                                follower.FollowPath(placeFourth,true),
                                intk.SetTwistPos(90),
                                intk.SetElbowPos(115),
                                intk.SetTrunkPos(95),
                                new SequentialAction(
                                        lift.setVertLifterPos(1150,1),
                                        follower.waitForPose(place4)
                                )
                        ),
                        lift.setVertLifterPos(1700,1),
                        intk.SetClawOpen(),
                        new ParallelAction(
                                follower.FollowPath(grabFourth,true),
                                intk.SetElbowPos(300),
                                intk.SetTwistPos(90),
                                intk.SetTrunkPos(60),
                                new SequentialAction(
                                        lift.setVertLifterPos(240,1),
                                        follower.waitForPose(grab)
                                )
                        ),
                        intk.SetClawClose(),
                        new SleepAction(.3),
                        new ParallelAction(
                                follower.FollowPath(placeFifth,true),
                                intk.SetTwistPos(90),
                                intk.SetElbowPos(115),
                                intk.SetTrunkPos(95),
                                new SequentialAction(
                                        lift.setVertLifterPos(1150,1),
                                        follower.waitForPose(place5)
                                )
                        ),
                        lift.setVertLifterPos(1700,1),
                        intk.SetClawOpen()
                )
            )
        );
    }

}
