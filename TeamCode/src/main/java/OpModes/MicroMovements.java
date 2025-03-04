package OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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


@Autonomous(name = "Micro Move Test", group = "Auto Testing")
public final class MicroMovements extends LinearOpMode {

    @Config
    public static class MicroMove {
        public static double DISTANCE = 10;
    }
    MicroMove params = new MicroMove();
    private MultipleTelemetry telemetryA;
    PathChain move1, move2, slideBlocks,grabSecond,placeSecond,grabThird,placeThird,grabFourth,placeFourth,camGrab;
    Drive follower;

    Pose startPose = new Pose(0, 0, Math.toRadians(0));
    Pose pos1 = new Pose(params.DISTANCE, 0, Math.toRadians(0));

    //All paths stored here
    public void buildPaths(){
        move1 = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(startPose),
                                new Point(pos1)
                        )


                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
        .build();
        move2 = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(pos1),
                                new Point(startPose)
                        )


                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
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

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

//        Actions.runBlocking(
//           //Follower Class loop
//            new ParallelAction(
//                follower.Update(telemetryA),
//                //main sequential
//                new SequentialAction(
//                        follower.FollowPath(move1,true),
//                        follower.waitForPose(pos1),
//                        follower.FollowPath(move2,true),
//                        follower.waitForPose(startPose),
//                        follower.StopUpdate()
//
//                )
//            )
//        );
    }

}
