package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.sections.Camera;
import org.firstinspires.ftc.teamcode.sections.Intake;
import org.firstinspires.ftc.teamcode.sections.Lifters;


@Autonomous(name = "Mag Test", group = "Auto Testing")
public final class MagTest extends LinearOpMode {


    //All paths stored here

    @Override
    public void runOpMode() throws InterruptedException {
        Lifters lift = new Lifters(hardwareMap);
        Intake intk = new Intake(hardwareMap);
        Camera cam = new Camera(hardwareMap,false);
        waitForStart();
        while(opModeIsActive()){
            if(lift.magR.isPressed()){
                telemetry.addData("R","yes");
            }else{
                telemetry.addData("R","no");
            }

            if(lift.magL.isPressed()){
                telemetry.addData("L","yes");
            }else{
                telemetry.addData("L","no");
            }

            telemetry.update();
        }
    }

}
