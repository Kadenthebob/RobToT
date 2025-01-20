package OpModes.Testing;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import sections.Drive;
import sections.Intake;
import sections.Lifters;


@Autonomous(name = "Set ZERO", group = "Auto Debug")
public final class HumanPlayerAuto extends LinearOpMode {

    PathChain placeFirst,hockeyFirstPlace,grabFirst,hockeySecondPlace,placeSecond,grabSecond,grabThird,hockeyFirst,hockeySecond,hockeyThird,grabFourth,placeFifth,placeThird,placeFourth,hockeyThirdPlace;
    Drive follower;

    @Override
    public void runOpMode() throws InterruptedException {

        Lifters lift = new Lifters(hardwareMap);
        Intake intk = new Intake(hardwareMap);

        waitForStart();
        intk.setClawPos(0);
        intk.setElbowPos(15);
        intk.setTrunkPos(190);
        while(opModeIsActive()){

        }

    }

}
