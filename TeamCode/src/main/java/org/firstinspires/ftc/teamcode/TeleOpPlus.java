package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants;
import org.firstinspires.ftc.teamcode.sections.Camera;
import org.firstinspires.ftc.teamcode.sections.Intake;
import org.firstinspires.ftc.teamcode.sections.Lifters;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

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
    MecanumDrive drive;
    Lifters lift;
    Intake intk;
    Follower follower;
    Camera cam;
    double xMov, yMov = 0;

    @Override
    public void runOpMode(){
        lift = new Lifters(hardwareMap);
        intk = new Intake(hardwareMap);
        cam = new Camera(hardwareMap,true);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(0,0,0));


        follower.resetIMU();

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
        lift.lifterHoldOff();
    }


    public void looping() {
        TelemetryPacket packet = new TelemetryPacket();
        lift.lifterHold().run(packet);

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

        if(gamepad1.y&&AutoGrabAction.size()==0){
            runningActions.add(follower.AutoGrab(cam,intk,lift,true));
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

        double brakeCoeff = 1-gamepad1.right_trigger;
        follower.setTeleOpMovementVectors((-gamepad1.left_stick_y+yMov)*brakeCoeff, (-gamepad1.left_stick_x+xMov)*brakeCoeff, -gamepad1.right_stick_x, true);
        follower.update();

        //gamepad 2
        if(gamepad2.a){  //intake in
            intk.setClawPos(0);
        } else if(gamepad2.b) {  //intake out
            intk.setClawPos(65);
        }
        if(gamepad2.y){
            intk.setTwistMatchObjAngle(cam,!cam.getSurrounded());
        }


        //turn off when not touching those two buttons
        if (gamepad2.dpad_left&&ArmAction.size()==0){
            ArmAction.add(intk.SetTrunkHoop());
        }
        else if(gamepad2.dpad_up&&ArmAction.size()==0){
            ArmAction.add(intk.SetTrunkPit());
        }else if(gamepad2.dpad_down&&ArmAction.size()==0) {
            ArmAction.add(intk.SetTrunkWall());
        } else if(gamepad2.dpad_right&&ArmAction.size()==0){
            ArmAction.add(intk.SetSpec1());
        }
        else {  //lift using triggers
            if(Math.abs(-gamepad2.right_trigger+gamepad2.left_trigger)>0.01){
                lift.vertLifterR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lift.vertLifterL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lift.setVertLifterPower(-gamepad2.right_trigger+gamepad2.left_trigger);
            }else{
                lift.lifterOverideOff();
            }
        }

        intk.setTwistPower(gamepad2.left_stick_x); //twist using left joystick
        intk.setElbowPower(-gamepad2.right_stick_y);
        if(gamepad2.left_bumper){  //left bumper = extendo forward
            lift.setHorLifterPower(1);
        } else if(gamepad2.right_bumper) { //right bumper = extendo backward
            lift.setHorLifterPower(-1);
        }


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
        telemetry.addData("x", cam.getObjX());
        telemetry.addData("y", cam.getObjY());
        telemetry.addData("angle", cam.getObjRot());
        telemetry.addData("max X", FollowerConstants.xMovement);
        telemetry.addData("focus",cam.cam.getFocusControl().getMinFocusLength());
        telemetry.addData("trigger",gamepad1.right_trigger);

        telemetry.update();

    }
}