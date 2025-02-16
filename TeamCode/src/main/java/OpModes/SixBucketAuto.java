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
    Pose grab1 = new Pose(18.75, 124, Math.toRadians(0));
    Pose grab2 = new Pose(18.75, 134, Math.toRadians(0));
    Pose grab3 = new Pose(22.5, 134, Math.toRadians(20));
    Pose grabAuto = new Pose(62.544, 96, Math.toRadians(-90));
    Pose place1 = new Pose(13, 131,Math.toRadians(-45));
    //Pose place2 = new Pose(14.793, 129.207, Math.toRadians(-45));



    //All paths stored here
    public void buildPaths(){

        placeFirst = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(startPose),
                                new Point(23.000, 120.000, Point.CARTESIAN),
                                new Point(place1)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .setZeroPowerAccelerationMultiplier(3)
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
                        new BezierCurve(
                                new Point(grab1),
                                new Point(20,130,Point.CARTESIAN),
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
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(20))
                .build();
        placeFourth = follower.pathBuilder()
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(grab3),
                                new Point(place1)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(-45))
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

        lift.setHorLifterPos(0);
        intk.setClawPos(0);
        intk.setTwistPos(90);
        //Aproach Pole
        intk.setTrunkPos(90);
        intk.setElbowPos(285);
        waitForStart();

        Actions.runBlocking(
                //Follower Class loop
                new ParallelAction(
                        follower.Update(telemetryA),
                        //main sequential
                        new SequentialAction(
                                new ParallelAction(
                                        follower.FollowPath(placeFirst,true),
                                        new SequentialAction(
                                                lift.setVertLifterPos(3400,1),
                                                intk.SetTrunkPos(60),
                                                intk.SetElbowPos(300),
                                                new SleepAction(.5),
                                                intk.SetClawOpen(),
                                                follower.waitForPose(place1)
                                        )
                                ),


                                new ParallelAction(
                                        follower.FollowPath(grabSecond,true),
                                        new SequentialAction(
                                                lift.SetHorLifterPos(105),
                                                intk.SetTrunkPit(),
                                                intk.SetClawOpen(),
                                                intk.SetTwistPos(90)
                                        ),
                                        new SequentialAction(
                                                lift.setVertLifterZero(1),
                                                follower.waitForPose(grab1)
                                        )
                                ),
                                new SleepAction(.3),
                                intk.SetClawClose(),
                                new SleepAction(.2),
                                lift.SetHorLifterPos(0),
                                intk.SetTrunkPos(90),
                                intk.SetElbowPos(285),

                                new ParallelAction(
                                        follower.FollowPath(placeSecond,true),
                                        new SequentialAction(
                                                lift.setVertLifterPos(3400,1),
                                                intk.SetTrunkPos(60),
                                                intk.SetElbowPos(300),
                                                new SleepAction(.5),
                                                intk.SetClawOpen(),
                                                follower.waitForPose(place1)
                                        )
                                ),


                                new ParallelAction(
                                        follower.FollowPath(grabThird,true),
                                        new SequentialAction(
                                                lift.SetHorLifterPos(105),
                                                intk.SetTrunkPit(),
                                                intk.SetClawOpen(),
                                                intk.SetTwistPos(90)

                                        ),
                                        new SequentialAction(
                                                lift.setVertLifterZero(1),
                                                follower.waitForPose(grab2)
                                        )
                                ),
                                new SleepAction(.3),
                                intk.SetClawClose(),
                                new SleepAction(.2),
                                lift.SetHorLifterPos(0),
                                intk.SetTrunkPos(90),
                                intk.SetElbowPos(285),

                                new ParallelAction(
                                        follower.FollowPath(placeThird,true),
                                        new SequentialAction(
                                                lift.setVertLifterPos(3400,1),
                                                intk.SetTrunkPos(60),
                                                intk.SetElbowPos(300),
                                                new SleepAction(.5),
                                                intk.SetClawOpen(),
                                                follower.waitForPose(place1)
                                        )
                                ),

                                new ParallelAction(
                                        follower.FollowPath(grabFourth,true),
                                        new SequentialAction(
                                                lift.SetHorLifterPos(80),
                                                intk.SetTrunkPit(),
                                                intk.SetTwistPos(110),
                                                intk.SetClawOpen()


                                        ),
                                        new SequentialAction(
                                                lift.setVertLifterZero(1),
                                                follower.waitForPose(grab3)

                                        )
                                ),
                                new SleepAction(.3),
                                intk.SetClawClose(),
                                new SleepAction(.2),
                                lift.SetHorLifterPos(0),
                                intk.SetTrunkPos(90),
                                intk.SetElbowPos(285),
                                intk.SetTwistPos(90),
                                new ParallelAction(
                                        follower.FollowPath(placeFourth,true),
                                        new SequentialAction(
                                                lift.setVertLifterPos(3400,1),
                                                intk.SetTrunkPos(60),
                                                intk.SetElbowPos(300),
                                                new SleepAction(.5),
                                                intk.SetClawOpen(),
                                                follower.waitForPose(place1)
                                        )
                                ),


                                new ParallelAction(
                                        follower.FollowPath(grabPit,true),
                                        new SequentialAction(
                                                lift.SetHorLifterPos(110),
                                                intk.SetTrunkPit(),
                                                intk.SetClawOpen(),
                                                intk.SetElbowPos(23),
                                                intk.SetTrunkPos(175),
                                                intk.SetTwistPos(90)

                                        ),
                                        new SequentialAction(
                                                lift.setVertLifterPos(800,1),
                                                follower.waitForPose(grabAuto)
                                        )
                                ),

                                follower.AutoGrab(cam,intk,lift),
                                intk.SetElbowPos(110),
                                new ParallelAction(
                                        lift.setVertLifterPos(300,1),
                                        follower.goToPose(grabAuto)
                                ),

                                follower.waitForPose(grabAuto),
                                lift.SetHorLifterPos(0),
                                intk.SetTrunkPos(90),
                                intk.SetElbowPos(285),
                                new ParallelAction(
                                        follower.FollowPath(placePit,true),
                                        new SequentialAction(
                                                lift.setVertLifterPos(3400,1),
                                                intk.SetTrunkPos(60),
                                                intk.SetElbowPos(300),
                                                new SleepAction(.5),
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
