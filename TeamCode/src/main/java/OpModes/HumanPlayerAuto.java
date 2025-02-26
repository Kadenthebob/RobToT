package OpModes;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.*;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import sections.*;


@Autonomous(name = "HP Auto", group = "Auto Testing")
public final class HumanPlayerAuto extends LinearOpMode {

    PathChain placeFirst,hockeyFirstPlace,grabFirst,hockeySecondPlace,placeSecond,grabSecond,grabThird,hockeyFirst,hockeySecond,hockeyThird,grabFourth,placeFifth,placeThird,placeFourth,hockeyThirdPlace;
    Drive follower;

    Pose startPose = new Pose(7.59375, 48, Math.toRadians(0));
    Pose hockey1 = new Pose(30.75, 36, Math.toRadians(-45));
    Pose hockey15 = new Pose(26, 36, Math.toRadians(-150));
    Pose hockey2 = new Pose(30.75, 27, Math.toRadians(-45));
    Pose hockey25 = new Pose(28, 27, Math.toRadians(-150));
    Pose hockey3 = new Pose(38, 12.5, Math.toRadians(-55));
    Pose grab1 = new Pose(17,25, Math.toRadians(0));
    Pose grab = new Pose(7.5,30, Math.toRadians(0));
    Pose forward = new Pose(7.6,30, Math.toRadians(0));
    Pose place1 = new Pose(32.25, 73, Math.toRadians(0));
    Pose place2 = new Pose(32.5, 72, Math.toRadians(0));
    Pose place3 = new Pose(32.5, 71, Math.toRadians(0));
    Pose place4 = new Pose(32.5, 69, Math.toRadians(0));
    Pose place5 = new Pose(32.5, 68, Math.toRadians(0));



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
                                new Point(28,73,Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(28,73,Point.CARTESIAN),
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
                .setLinearHeadingInterpolation(Math.toRadians(-150), Math.toRadians(-55))
                .build();

        hockeyThirdPlace = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(hockey3),
                                new Point(grab1)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-55), Math.toRadians(0))
                .build();

        grabFirst = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(grab1),
                                new Point(grab)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        placeSecond = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(grab),
                                new Point(place2)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        grabSecond = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(place2),
                                new Point(grab)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        placeThird = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(grab),
                                new Point(place3)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        grabThird = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(place3),
                                new Point(grab)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        placeFourth = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(grab),
                                new Point(place4)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        grabFourth = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(place4),
                                new Point(grab)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        placeFifth= follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(grab),
                                new Point(place5)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

    }

    @Override
    public void runOpMode() throws InterruptedException {

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Drive(hardwareMap);
        follower.poseUpdater.resetIMU();
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
                        lift.SetHorLifterPos(90),
                        intk.SetTwistPos(90),
                        intk.SetElbowPos(115),
                        intk.SetTrunkPos(95),
                        intk.SetClawClose(),
                        new ParallelAction(
                                follower.FollowPath(placeFirst),

                                new SequentialAction(
                                        lift.setVertLifterPos(1150,1),
                                        follower.waitForPoint(place1)
                                )
                        ),
                        lift.setVertLifterPos(1900,1),
                        intk.SetClawOpen(),

                        new ParallelAction(
                                follower.FollowPath(hockeyFirst,true),
                                        new SleepAction(2),
                                        //.53
                                        intk.SetTrunkPit(),
                                        intk.SetClawOpen(),
                                        intk.SetTwistPos(45),//.0815
                                new SequentialAction(
                                        new SleepAction(1),
                                        lift.setVertLifterPos(450,1),
                                        follower.waitForPose(hockey1),
                                        lift.setVertLifterZero(1)

                                )
                        ),

                        intk.SetClawClose(),
                        new SleepAction(.1),
                        new ParallelAction(
                                follower.FollowPath(hockeyFirstPlace,true),
                                new SequentialAction(
                                        lift.setVertLifterPos(450,1),
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
                        new SleepAction(.3),
                        lift.setVertLifterZero(1),
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
                                lift.SetHorLifterPos(0),
                                intk.SetTwistPos(30),
                                new SequentialAction(
                                        follower.waitForPose(hockey3)

                                )
                        ),
                        lift.setVertLifterZero(1),
                        intk.SetClawClose(),
                        new SleepAction(.3),
                        new ParallelAction(
                                follower.FollowPath(hockeyThirdPlace,true),
                                new SequentialAction(
                                        lift.SetHorLifterPos(0), //.53
                                        intk.SetTrunkWall(), //.374
                                        intk.SetClawOpen()
                                ),
                                new SequentialAction(
                                        follower.waitForPose(grab1)

                                )
                        ),
                        new ParallelAction(
                                follower.FollowPath(grabFirst,true),
                                intk.SetElbowPos(300),
                                intk.SetTwistPos(90),
                                intk.SetTrunkPos(60),
                                new SequentialAction(
                                        follower.waitForPose(grab)
                                )
                        ),
                        intk.SetClawClose(),
                        new SleepAction(.15),
                        new ParallelAction(
                                follower.FollowPath(placeSecond,true),
                                lift.SetHorLifterPos(90),
                                intk.SetTwistPos(90),
                                intk.SetElbowPos(115),
                                intk.SetTrunkPos(95),
                                new SequentialAction(
                                        lift.setVertLifterPos(1150,1),
                                        follower.waitForPose(place2)
                                )
                        ),
                        lift.setVertLifterPos(1800,1),
                        intk.SetClawOpen(),
                        new ParallelAction(
                                follower.FollowPath(grabSecond,true),
                                lift.SetHorLifterPos(0),
                                intk.SetElbowPos(300),
                                intk.SetTwistPos(90),
                                intk.SetTrunkPos(60),
                                new SequentialAction(
                                        new SleepAction(1),
                                        lift.setVertLifterZero(1),
                                        follower.waitForPose(grab)
                                )
                        ),
                        intk.SetClawClose(),
                        new SleepAction(.15),
                        new ParallelAction(
                                follower.FollowPath(placeThird,true),
                                lift.SetHorLifterPos(90),
                                intk.SetTwistPos(90),
                                intk.SetElbowPos(115),
                                intk.SetTrunkPos(95),
                                new SequentialAction(
                                        lift.setVertLifterPos(1150,1),
                                        follower.waitForPose(place3)
                                )
                        ),
                        lift.setVertLifterPos(1800,1),
                        intk.SetClawOpen(),
                        new ParallelAction(
                                follower.FollowPath(grabThird,true),
                                lift.SetHorLifterPos(0),
                                intk.SetElbowPos(300),
                                intk.SetTwistPos(90),
                                intk.SetTrunkPos(60),
                                new SequentialAction(
                                        new SleepAction(1),
                                        lift.setVertLifterZero(1),
                                        follower.waitForPose(grab)
                                )
                        ),
                        intk.SetClawClose(),
                        new SleepAction(.15),
                        new ParallelAction(
                                follower.FollowPath(placeFourth,true),
                                lift.SetHorLifterPos(90),
                                intk.SetTwistPos(90),
                                intk.SetElbowPos(115),
                                intk.SetTrunkPos(95),
                                new SequentialAction(
                                        lift.setVertLifterPos(1150,1),
                                        follower.waitForPose(place4)
                                )
                        ),
                        lift.setVertLifterPos(1800,1),
                        intk.SetClawOpen(),
                        new ParallelAction(
                                follower.FollowPath(grabFourth,true),
                                lift.SetHorLifterPos(0),
                                intk.SetElbowPos(300),
                                intk.SetTwistPos(90),
                                intk.SetTrunkPos(60),
                                new SequentialAction(
                                        new SleepAction(1),
                                        lift.setVertLifterZero(1),
                                        follower.waitForPose(grab)
                                )
                        ),
                        intk.SetClawClose()

                )
            )
        );
    }

}
