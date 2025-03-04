package OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Tele-OP")
public class TeleOpPlus extends LinearOpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    private List<Action> AutoGrabAction = new ArrayList<>();
    private List<Action> ArmAction = new ArrayList<>();
    boolean driverOveride = false;
    boolean clickedX = false;
    Lifters lift;
    Intake intk;
    Drive follower;
//    Camera cam;
    double xMov, yMov = 0;
    boolean holdingA = false;
    boolean holdingY = false;

    @Override
    public void runOpMode() throws InterruptedException{
        lift = new Lifters(hardwareMap);
        intk = new Intake(hardwareMap);
//        cam = new Camera(hardwareMap,true);
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Drive(hardwareMap);
        follower.poseUpdater.resetIMU();
        follower.setStartingPose(new Pose(0,0,0));
//        drive.setTeamRed();
//        drive.pauseCamera();
        waitForStart();
        follower.startTeleopDrive();
        intk.setElbowPos(0);
        intk.setTwistPos(90);
        lift.setHorLifterPos(0);

        ArmAction.add(intk.SetTrunkHoop());
//        drive.readPos();
        while(opModeIsActive()){
            looping();
        }
    }


    public void looping() {
        TelemetryPacket packet = new TelemetryPacket();

        // update running actions
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        List<Action> newAutoGrab = new ArrayList<>();
        for (Action action : AutoGrabAction) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newAutoGrab.add(action);
            }
        }
        AutoGrabAction = newActions;

        List<Action> newArmAction = new ArrayList<>();
        for (Action action : ArmAction) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newArmAction.add(action);
            }
        }
        ArmAction = newArmAction;

        //auto grab code

//        if(gamepad1.y&&AutoGrabAction.size()==0){
//            runningActions.add(follower.AutoGrab(cam,intk,lift,true));
//        }
        if(gamepad1.b&&AutoGrabAction.size()!=0){
            runningActions.set(0,intk.autoOverideOff());
            follower.startTeleopDrive();
        }

        if(!driverOveride) {
            if (gamepad1.dpad_up) {
                yMov=1;
            } else if (gamepad1.dpad_down) {
                yMov=-1;
            } else if (gamepad1.dpad_left){
                xMov=1;
            } else if (gamepad1.dpad_right){
                xMov=-1;
            }
            else {
                xMov = 0;
                yMov = 0;
            }
        }
        double brakeCoeff;
        if(gamepad1.right_bumper){
            brakeCoeff = .3;
        }else brakeCoeff = 1-gamepad1.right_trigger;

//        if(!follower.getTeleOpOveride(cam)) {
            follower.setTeleOpMovementVectors((-gamepad1.left_stick_y + yMov) * brakeCoeff, (-gamepad1.left_stick_x + xMov) * brakeCoeff, -gamepad1.right_stick_x, true);
//        }
        follower.update();

//        intk.setTrunkPower(-gamepad2.left_stick_y);

        //gamepad 2

        if(gamepad2.a&&!holdingA){
            holdingA = true;//intake in
            intk.toggleClaw();
        } else if(!gamepad2.a){
            holdingA = false;
        }


        //arm actions
        //camera adjuster
        if(gamepad2.back){
//            if(gamepad2.x){
//                cam.setDetectYellow();
//            }else if(gamepad2.y){
//                cam.setDetectBoth();
//            }else if(gamepad2.b){
//                cam.setDetectColor();
            if(gamepad2.a && ArmAction.size() == 0){
                ArmAction.add(lift.setVertLifterZero(1));
            }
        }else {
            if (gamepad2.x && ArmAction.size() == 0) {
                ArmAction.add(new ParallelAction(intk.SetTrunkHoop(), lift.setVertLifterPos(3400, 1)));
            } else if (gamepad2.dpad_up && ArmAction.size() == 0) {
                ArmAction.add(new ParallelAction(intk.SetTrunkPit(), lift.setVertLifterPos(700, 1)));
            } else if (gamepad2.dpad_down && ArmAction.size() == 0) {
                ArmAction.add(intk.SetTrunkWall());
            }

            if(gamepad2.y&&!holdingY&&ArmAction.size()==0){
                holdingY = true;
                ArmAction.add(intk.SetSpec(lift));
            } else if(!gamepad2.y&&holdingY){
                holdingY = false;
                try{
                    ArmAction.set(0,intk.SetSpecRelease(lift));
                } catch(Exception e){
                    ArmAction.add(intk.SetSpecRelease(lift));
                }

            }
        }



         //lift using triggers
        if(Math.abs(-gamepad2.right_trigger+gamepad2.left_trigger)>0.01){
            lift.setVertLifterPower(-gamepad2.right_trigger+gamepad2.left_trigger);
        }else{
            lift.lifterHold().run(packet);
        }

        if(ArmAction.size()==0) {
            if (gamepad2.left_stick_x > .1) {
                intk.setTwistPos(145);
            }else if (gamepad2.left_stick_x < -.1) {
                intk.setTwistPos(35);
            }else if (gamepad2.b){
                intk.setTwistPos(0);
            } else {
                intk.setTwistPos(90);
            }
        }
        //twist using left joystick
        intk.setElbowPower(-gamepad2.right_stick_y);
        if(gamepad2.left_bumper){  //left bumper = extendo forward
            lift.setHorLifterPower(1);
        } else if(gamepad2.right_bumper) { //right bumper = extendo backward
            lift.setHorLifterPower(-1);
        }



//        telemetry.addData("Cam Detect Setting", cam.getDetectSettings());
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addData("Driver Overide: ", driverOveride);  //print out data
        telemetry.addData("X: ", follower.getPose().getX());
        telemetry.addData("Y: ", follower.getPose().getY());
        telemetry.addData("Heading: ", follower.getPose().getHeading());
        telemetry.addData("Lifter R: ", lift.vertLifterR.getCurrentPosition());
        telemetry.addData("Lifter L: ", lift.vertLifterL.getCurrentPosition());
        telemetry.addData("Twist Pos: ", intk.twist.getPosition());
        telemetry.addData("Trunk Pos: ", intk.trunkR.getPosition());
        telemetry.addData("extendoL Pos: ", lift.horLifterL.getPosition());
        telemetry.addData("extendoR Pos: ", lift.horLifterR.getPosition());
        telemetry.addData("arm: ", intk.arm.getPosition());
//        telemetry.addData("focus",cam.cam.getFocusControl().getMinFocusLength());

        telemetry.update();
    }
}