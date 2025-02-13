package OpModes.Testing;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.localization.Pose;
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


@Autonomous(name = "Auto Twist", group = "Auto Debug")
public final class AutoTwistTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Intake intk = new Intake(hardwareMap);
        Camera cam = new Camera(hardwareMap,false);
        waitForStart();
        boolean holdingA = false;
        while(opModeIsActive()){
            if(gamepad1.a&&!holdingA) {
                holdingA = true;
                intk.setTwistMatchObjAngle(cam);
            } else if(!gamepad1.a){
                holdingA = false;
            }
        }
    }

}
