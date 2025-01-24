package OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.*;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import sections.*;


@Autonomous(name = "Six Bucket Auto", group = "Auto Testing")
public final class SixBucketAuto extends LinearOpMode {

    PathChain placeFirst,grabSecond,placeSecond,grabThird,placeThird,grabFourth,placeFourth,grabPit,placePit,forward;
    Drive follower;

    private MultipleTelemetry telemetryA;

    Pose startPose = new Pose(7.59375, 96, Math.toRadians(0));
    Pose grab1 = new Pose(32, 121, Math.toRadians(0));
    Pose grab2 = new Pose(32, 130.5, Math.toRadians(0));
    Pose grab3 = new Pose(41.000, 123.106, Math.toRadians(80));
    Pose grabAuto = new Pose(62.544, 98.497, Math.toRadians(-90));
    Pose forward1 = new Pose(12.793,129.207,Math.toRadians(-45));
    Pose place1 = new Pose(14, 129.207, Math.toRadians(-45));
    //Pose place2 = new Pose(14.793, 129.207, Math.toRadians(-45));



    //All paths stored here
    public void buildPaths(){

        placeFirst = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(startPose),
                                new Point(20.000, 110.000, Point.CARTESIAN),
                                new Point(place1)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .build();

        forward = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(place1),
                                new Point(forward1)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-45))
                .build();



        grabSecond = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(place1),
                                new Point(grab1)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .build();

        placeSecond = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(grab1),
                                new Point(place1)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .build();

        grabThird = follower.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(place1),
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
                                new Point(place1)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .build();

        grabFourth = follower.pathBuilder()
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(place1),
                                new Point(grab3)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(80))
                .build();
        placeFourth = follower.pathBuilder()
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(grab3),
                                new Point(place1)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(80), Math.toRadians(-45))
                .build();

        grabPit = follower.pathBuilder()
                .addPath(
                        // Line 8
                        new BezierCurve(
                                new Point(place1),
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
                                new Point(place1)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-45))
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
        Camera cam = new Camera(hardwareMap,true);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        Actions.runBlocking(
           //Follower Class loop
            new ParallelAction(
                follower.Update(telemetryA),
                //main sequential
                new SequentialAction(
                        intk.SetClawClose(),
                        intk.SetTwistPos(90),
                        //Aproach Pole
                        new ParallelAction(
                                follower.FollowPath(placeFirst,true),
                                intk.SetTrunkWall(),
                                new SequentialAction(
                                        lift.setVertLifterPos(4300,1),
                                        intk.SetClawOpen(),
                                        follower.waitForPose(place1)
                                )
                        ),



                        new ParallelAction(
                                follower.FollowPath(grabSecond,true),
                                new SequentialAction(
                                        intk.SetTrunkPit(),
                                        intk.SetClawOpen(),
                                        intk.SetTwistPos(90)
                                ),
                                new SequentialAction(
                                        lift.setVertLifterZero(1),
                                        follower.waitForPose(grab1)
                                )
                        ),
                        intk.SetClawClose(),
                        new SleepAction(.5),

                        new ParallelAction(
                                follower.FollowPath(placeSecond,true),
                                intk.SetTrunkWall(),
                                new SequentialAction(
                                        lift.setVertLifterPos(4300,1),
                                        intk.SetClawOpen(),
                                        follower.waitForPose(place1)
                                )
                        ),

                        intk.SetClawOpen(),

                        new ParallelAction(
                                follower.FollowPath(grabThird,true),
                                new SequentialAction(
                                        intk.SetTrunkPit(),
                                        intk.SetClawOpen(),
                                        intk.SetTwistPos(90)

                                ),
                                new SequentialAction(
                                        lift.setVertLifterZero(1),
                                        follower.waitForPose(grab2)
                                )
                        ),
                        intk.SetClawClose(),
                        new SleepAction(.3),


                        new ParallelAction(
                                follower.FollowPath(placeThird,true),
                                intk.SetTrunkWall(),
                                new SequentialAction(
                                        lift.setVertLifterPos(4300,1),
                                        intk.SetClawOpen(),
                                        follower.waitForPose(place1)
                                )
                        ),


                        lift.SetHorLifterPos(48),
                        new ParallelAction(
                                follower.FollowPath(grabFourth,true),
                                new SequentialAction(
                                        intk.SetTrunkPit(),
                                        intk.SetClawOpen()

                                ),
                                new SequentialAction(
                                        lift.setVertLifterZero(1),
                                        follower.waitForPose(grab3)

                                )
                        ),
                        new SleepAction(.3),
                        intk.SetClawClose(),
                        new SleepAction(.3),
                        lift.SetHorLifterPos(0),
                        new ParallelAction(
                                follower.FollowPath(placeFourth,true),
                                intk.SetTrunkWall(),
                                new SequentialAction(
                                        lift.setVertLifterPos(4300,1),
                                        intk.SetClawOpen(),
                                        follower.waitForPose(place1)
                                )
                        ),


                        new ParallelAction(
                                follower.FollowPath(grabPit,true),
                                new SequentialAction(
                                        lift.SetHorLifterPos(90),
                                        intk.SetTrunkPit(),
                                        intk.SetClawOpen(),
                                        intk.SetTwistPos(0)


                                ),
                                new SequentialAction(
                                        lift.setVertLifterPos(700,1),
                                        follower.waitForPose(grabAuto)
                                )
                        ),
                        new SleepAction(1),
                        follower.AutoGrab(cam,intk,lift),
                        follower.goToPose(grabAuto),
                        follower.waitForPose(grabAuto),
                        lift.SetHorLifterPos(0),
                        new ParallelAction(
                                follower.FollowPath(placePit,true),
                                intk.SetTrunkWall(),
                                new SequentialAction(
                                        lift.setVertLifterPos(4300,1),
                                        intk.SetClawOpen(),
                                        follower.waitForPose(place1)
                                )
                        ),



                        new ParallelAction(
                                follower.FollowPath(grabPit,true),
                                intk.SetTrunkWall(),
                                new SequentialAction(
                                        lift.setVertLifterPos(50,1),
                                        follower.waitForPose(grabAuto)
                                )
                        ),
                        new SleepAction(1)
                )
            )
        );
    }

}
