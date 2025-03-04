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


@Autonomous(name = "Bucket Auto", group = "Auto Testing")
public final class BucketAuto extends LinearOpMode {

    PathChain placeFirst,grabSecond,placeSecond,grabThird,placeThird,grabFourth,placeFourth,grabPit,placePit;
    Drive follower;

    Pose startPose = new Pose(7.59375, 96, Math.toRadians(0));
    Pose grab1 = new Pose(29.680, 120.967, Math.toRadians(0));
    Pose grab2 = new Pose(29.680, 131.267, Math.toRadians(0));
    Pose grab3 = new Pose(37.000, 122.106, Math.toRadians(70));
    Pose grabAuto = new Pose(62.544, 98.497, Math.toRadians(-90));
    Pose grabCP = new Pose(28,24,Math.toRadians(0));

    Pose place1 = new Pose(38.000, 76.588, Math.toRadians(0));
    Pose place2 = new Pose(14.793, 129.207, Math.toRadians(-45));

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
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();


        grabSecond = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(place1),
                                new Point(30.000, 100.000, Point.CARTESIAN),
                                new Point(grab1)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        placeSecond = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(grab1),
                                new Point(place2)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .build();

        grabThird = follower.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(place2),
                                new Point(grab2)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .build();

        placeThird = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(grab2),
                                new Point(place2)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .build();

        grabFourth = follower.pathBuilder()
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(place2),
                                new Point(grab3)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(70))
                .build();
        placeFourth = follower.pathBuilder()
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(grab3),
                                new Point(place2)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(-45))
                .build();

        grabPit = follower.pathBuilder()
                .addPath(
                        // Line 8
                        new BezierCurve(
                                new Point(place2),
                                new Point(65.000, 115.000, Point.CARTESIAN),
                                new Point(grabAuto)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-90))
                .build();

        placePit = follower.pathBuilder()
                .addPath(
                        // Line 9
                        new BezierCurve(
                                new Point(grabAuto),
                                new Point(70.000, 105.000, Point.CARTESIAN),
                                new Point(place2)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-45))
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
                                        follower.waitForPose(place1)
                                )
                        ),


                        new ParallelAction(
                                follower.FollowPath(grabSecond),
                                new SequentialAction(
                                        follower.waitForPose(grab1)
                                )
                        ),


                        new ParallelAction(
                                follower.FollowPath(placeSecond,true),
                                new SequentialAction(
                                        lift.setVertLifterPos(4000,1),
                                        follower.waitForPose(place2)
                                )
                        ),

                        new ParallelAction(
                                follower.FollowPath(grabThird,true),
                                new SequentialAction(
                                        lift.setVertLifterPos(0,1),
                                        follower.waitForPose(grab2)
                                )
                        ),


                        new ParallelAction(
                                follower.FollowPath(placeThird,true),
                                new SequentialAction(
                                        lift.setVertLifterPos(4000,1),
                                        follower.waitForPose(place2)
                                )
                        ),

                        lift.SetHorLifterPos(90),
                        new ParallelAction(
                                follower.FollowPath(grabFourth,true),
                                new SequentialAction(
                                        lift.setVertLifterPos(0,1),
                                        follower.waitForPose(grab3)
                                )
                        ),
                        lift.SetHorLifterPos(0),
                        new ParallelAction(
                                follower.FollowPath(placeFourth,true),
                                new SequentialAction(
                                        lift.setVertLifterPos(4000,1),
                                        follower.waitForPose(place2)
                                )
                        ),

                        new ParallelAction(
                                follower.FollowPath(grabPit),
                                new SequentialAction(
                                        lift.setVertLifterPos(0,1),
                                        follower.waitForPose(grabAuto)
                                )
                        ),


                        new ParallelAction(
                                follower.FollowPath(placePit),
                                new SequentialAction(
                                        lift.setVertLifterPos(4000,1),
                                        follower.waitForPose(place2)
                                )
                        ),

                        new SleepAction(1)
                )
            )
        );
    }

}
