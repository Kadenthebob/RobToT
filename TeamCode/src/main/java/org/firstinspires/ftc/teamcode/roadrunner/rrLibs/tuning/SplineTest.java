package org.firstinspires.ftc.teamcode.roadrunner.rrLibs.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.rrLibs.unused.TankDrive;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
//
public final class SplineTest extends LinearOpMode {
    private MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(new SequentialAction(
                drive.actionBuilder(beginPose)
                        .splineTo(new Vector2d(20, 20), Math.PI / 2)
                        .splineTo(new Vector2d(0, 40), Math.PI)
                        .splineTo(new Vector2d(20, 20), Math.PI / 2)
                        .splineTo(new Vector2d(0, 0), 0)
                        .build())
            );
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                            .splineTo(new Vector2d(0, 60), Math.PI)
                            .build());
        } else {
            throw new RuntimeException();
        }
    }
}
