package OpModes.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.Localizers;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.localization.localizers.OTOSLocalizer;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import sections.Camera;
import sections.Drive;
import sections.Intake;
import sections.Lifters;


@Autonomous(name = "RR&PP Strait Back and Forth", group = "Auto Debug")
public final class StraitBnF extends LinearOpMode {

    PathChain forward, backward;
    Drive follower;
    private MultipleTelemetry telemetryA;

    Pose startPose = new Pose(0, 0, Math.toRadians(0));
    Pose endPose = new Pose(48, 0, Math.toRadians(0));
    public void buildPaths(){

        forward = follower.pathBuilder()
                    .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(startPose),
                                new Point(endPose)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        backward = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(endPose),
                                new Point(startPose)
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

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        PoseUpdater pos = new PoseUpdater(hardwareMap, new OTOSLocalizer(hardwareMap));

        waitForStart();
        while(opModeIsActive()) {
            Actions.runBlocking(
                    new ParallelAction(
                            follower.Update(telemetryA),
                            new SequentialAction(
                                    follower.FollowPath(forward, true),
                                    follower.waitForPose(endPose),
                                    follower.FollowPath(backward, true),
                                    follower.waitForPose(startPose),
                                    follower.StopUpdate()
                            )
                    )
            );
        }
    }

}
