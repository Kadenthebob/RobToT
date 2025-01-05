package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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


@Autonomous(name = "Six Bucket Auto", group = "Auto Testing")
public final class SixBucketAuto extends LinearOpMode {

    PathChain placeFirst,grabSecond,placeSecond,grabThird,placeThird,grabFourth,placeFourth,grabPit,placePit;
    Follower follower;

    private MultipleTelemetry telemetryA;

    Pose startPose = new Pose(7.59375, 96, Math.toRadians(0));
    Pose grab1 = new Pose(32, 119.967, Math.toRadians(0));
    Pose grab2 = new Pose(32, 129.75, Math.toRadians(0));
    Pose grab3 = new Pose(42.000, 122.106, Math.toRadians(80));
    Pose grabAuto = new Pose(62.544, 98.497, Math.toRadians(-90));
    Pose grabCP = new Pose(28,24,Math.toRadians(0));

    Pose place1 = new Pose(14.793, 129.207, Math.toRadians(-45));
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


        follower = new Follower(hardwareMap);
        follower.resetIMU();
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
                        intk.SetTwistPos(90),
                        //Aproach Pole
                        new ParallelAction(
                                follower.FollowPath(placeFirst,true),
                                intk.SetTrunkWall(),
                                new SequentialAction(
                                        lift.setVertLifterPos(4000,1),
                                        follower.waitForPose(place1)
                                )
                        ),


                        new ParallelAction(
                                follower.FollowPath(grabSecond,true),
                                new SequentialAction(
                                        intk.SetTrunkPit(),
                                        intk.SetClawOpen(),
                                        intk.SetTwistPos(0)
                                ),
                                new SequentialAction(
                                        lift.setVertLifterPos(50,1),
                                        follower.waitForPose(grab1)
                                )
                        ),
                        intk.SetClawClose(),
                        new SleepAction(.5),

                        new ParallelAction(
                                follower.FollowPath(placeSecond,true),
                                intk.SetTrunkWall(),
                                new SequentialAction(
                                        lift.setVertLifterPos(4000,1),
                                        intk.SetClawOpen(),
                                        follower.waitForPose(place1)
                                )
                        ),

                        new ParallelAction(
                                follower.FollowPath(grabThird,true),
                                new SequentialAction(
                                        intk.SetTrunkPit(),
                                        intk.SetClawOpen(),
                                        intk.SetTwistPos(0)

                                ),
                                new SequentialAction(
                                        lift.setVertLifterPos(50,1),
                                        follower.waitForPose(grab2)
                                )
                        ),
                        intk.SetClawClose(),
                        new SleepAction(.3),


                        new ParallelAction(
                                follower.FollowPath(placeThird,true),
                                intk.SetTrunkWall(),
                                new SequentialAction(
                                        lift.setVertLifterPos(4000,1),
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
                                        lift.setVertLifterPos(50,1),
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
                                        lift.setVertLifterPos(4000,1),
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
                        follower.AutoGrab(cam,intk,lift),
                        follower.goToPose(grabAuto),
                        follower.waitForPose(grabAuto),
                        lift.SetHorLifterPos(0),
                        new ParallelAction(
                                follower.FollowPath(placePit,true),
                                intk.SetTrunkWall(),
                                new SequentialAction(
                                        lift.setVertLifterPos(4000,1),
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
