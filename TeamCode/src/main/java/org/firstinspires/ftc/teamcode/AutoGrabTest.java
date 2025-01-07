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
    Pose grab1 = new Pose(48, 0, Math.toRadians(0));
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
                                new Point(grab1)
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
        Camera cam = new Camera(hardwareMap,true);
        waitForStart();
        Actions.runBlocking(
           //Follower Class loop
            new ParallelAction(
                follower.Update(),
                //main sequential
                new SequentialAction(
                        lift.setVertLifterPos(700,1),
                        follower.WaitForDetect(cam),
                        follower.AutoMoveLoop(cam),
//                        intk.SetTrunkPit(),
//                        lift.SetHorLifterPos(90),
//                        new SleepAction(1),
//                        follower.AutoGrab(cam,intk,lift),
//                        follower.goToPose(new Pose(0,0,0)),
//                        intk.SetTrunkWall(),
//                        follower.waitForPose(new Pose(0,0,0)),
                        follower.StopUpdate()

                )
            )
        );
    }

}
