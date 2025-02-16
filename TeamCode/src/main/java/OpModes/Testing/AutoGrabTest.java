package OpModes.Testing;

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



@Autonomous(name = "Auto Grab", group = "Auto Debug")
public final class AutoGrabTest extends LinearOpMode {

    PathChain placeFirst, slideBlocks,grabSecond,placeSecond,grabThird,placeThird,grabFourth,placeFourth,camGrab;
    Drive follower;

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

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Drive(hardwareMap);
        follower.poseUpdater.resetIMU();
        buildPaths();
        follower.setStartingPose(startPose);

        Lifters lift = new Lifters(hardwareMap);
        Intake intk = new Intake(hardwareMap);
        Camera cam = new Camera(hardwareMap,false);
        waitForStart();
        Actions.runBlocking(
                new ParallelAction(
                        follower.Update(),
                        //main sequential
                        new SequentialAction(
                                lift.SetHorLifterPos(110),
                                lift.setVertLifterPos(800,1),
                                intk.SetClawOpen(),
                                intk.SetElbowPos(23),
                                intk.SetTrunkPos(175),
                                intk.SetTwistPos(90),
                                follower.AutoGrab(cam, intk, lift),
                                lift.setVertLifterPos(200,1),
                                follower.StopUpdate()

                        )
                )
           //Follower Class loop
//            new ParallelAction(
//                follower.Update(),
//                //main sequential
//                new SequentialAction(
//                        new ParallelAction(
//                                follower.goToPose(new Pose(5,0)),
//                                lift.setVertLifterPos(700,1)
//                        ),
//                        follower.waitForPose(new Pose(5,0)),
//                        follower.AutoGrab(cam,intk,lift),
////                        follower.goToPose(new Pose(0,0,0)),
////                        intk.SetTrunkWall(),
////                        follower.waitForPose(new Pose(0,0,0)),
//                        new SleepAction(1),
//                        follower.StopUpdate()
//
//                )
//            )
        );
    }

}
