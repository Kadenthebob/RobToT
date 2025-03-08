package OpModes;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.*;

import sections.*;


@Autonomous(name = "HP Auto", group = "Auto Testing")
public final class HumanPlayerAuto extends LinearOpMode {

    PathChain placeFirst,hockeyFirstPlace,grabFirst,hockeySecondPlace,placeSecond,grabSecond,grabThird,hockeyFirst,hockeySecond,hockeyThird,grabFourth,placeFifth,placeThird,placeFourth,hockeyThirdPlace;
    Drive follower;

    Pose startPose = new Pose(9, 48, Math.toRadians(0));
    Pose hockey1 = new Pose(65, 24, Math.toRadians(0));
    Pose hockey15 = new Pose(21, 24, Math.toRadians(0));
    Pose hockey2 = new Pose(65, 24, Math.toRadians(0));
    Pose hockey25 = new Pose(21, 14, Math.toRadians(0));
    Pose hockey3 = new Pose(65, 14, Math.toRadians(-55));
    Pose grab1 = new Pose(21,9, Math.toRadians(0));
    Pose grab = new Pose(9,24.5, Math.toRadians(0));
    Pose forward = new Pose(7.6,30, Math.toRadians(0));
    Pose place1 = new Pose(40, 69, Math.toRadians(0));
    Pose place2 = new Pose(40, 72, Math.toRadians(0));
    Pose place3 = new Pose(40, 71, Math.toRadians(0));
    Pose place4 = new Pose(40, 69, Math.toRadians(0));
    Pose place5 = new Pose(40, 68, Math.toRadians(0));



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
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        hockeyFirst = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(place1),
                                new Point(20,50,Point.CARTESIAN),
                                new Point(40,45,Point.CARTESIAN),
                                new Point(hockey1)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        hockeyFirstPlace = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(hockey1),
                            new Point(hockey15)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        hockeySecond = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(hockey15),
                                new Point(hockey2)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        hockeySecondPlace= follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierCurve(
                                new Point(hockey2),
                                new Point(70,10,Point.CARTESIAN),
                                new Point(hockey25)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        hockeyThird = follower.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(hockey25),
                                new Point(hockey3)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        hockeyThirdPlace = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(hockey3),
                                new Point(60,5,Point.CARTESIAN),
                                new Point(grab1)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
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

        follower = new Drive(hardwareMap);
        follower.poseUpdater.resetIMU();
        buildPaths();
        follower.setStartingPose(startPose);

        Lifters lift = new Lifters(hardwareMap);
        Intake intk = new Intake(hardwareMap);
        Outake out = new Outake(hardwareMap);
        intk.SetClawClose();
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
                                new SequentialAction(
                                        follower.waitForPoint(place1)
                                )
                        ),

                        new ParallelAction(
                                follower.FollowPath(hockeyFirst,true),
                                new SequentialAction(
                                        follower.waitForPose(hockey1)
                                )

                        ),

                        new ParallelAction(
                                follower.FollowPath(hockeyFirstPlace,true),
                                new SequentialAction(
                                        follower.waitForPose(hockey15)
                                )
                        ),
                        new ParallelAction(
                                follower.FollowPath(hockeySecond,true),
                                new SequentialAction(
                                        follower.waitForPose(hockey2)

                                )
                        ),
                        new ParallelAction(
                                follower.FollowPath(hockeySecondPlace,true),
                                new SequentialAction(
                                        follower.waitForPose(hockey25)

                                )
                        ),

                        new ParallelAction(
                                follower.FollowPath(hockeyThird,true),
                                new SequentialAction(
                                        follower.waitForPose(hockey3)

                                )
                        ),
                        new ParallelAction(
                                follower.FollowPath(hockeyThirdPlace,true),
                                new SequentialAction(
                                        follower.waitForPose(grab1)
                                )
                        ),
                        new ParallelAction(
                                follower.FollowPath(grabFirst,true),
                                intk.SetElbowPos(300),
                                intk.SetTwistPos(90),
                                intk.SetElbowPos(60),
                                new SequentialAction(
                                        follower.waitForPose(grab)
                                )
                        ),
                        intk.SetClawClose(),
                        new SleepAction(.15),
                        new ParallelAction(
                                follower.FollowPath(placeSecond,true),
                                lift.SetHorLifterPos(75),
                                intk.SetTwistPos(90),
                                intk.SetElbowPos(115),
                                intk.SetElbowPos(95),
                                new SequentialAction(
                                        lift.setVertLifterPos(1000,1),
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
                                intk.SetElbowPos(60),
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
                                lift.SetHorLifterPos(75),
                                intk.SetTwistPos(90),
                                intk.SetElbowPos(115),
                                intk.SetElbowPos(95),
                                new SequentialAction(
                                        lift.setVertLifterPos(1000,1),
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
                                intk.SetElbowPos(60),
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
                                lift.SetHorLifterPos(75),
                                intk.SetTwistPos(90),
                                intk.SetElbowPos(115),
                                intk.SetElbowPos(95),
                                new SequentialAction(
                                        lift.setVertLifterPos(1000,1),
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
                                intk.SetElbowPos(60),
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
