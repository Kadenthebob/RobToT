package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.sections.Intake;
import org.firstinspires.ftc.teamcode.sections.Lifters;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Tele-OP")
public class TeleOpPlus extends LinearOpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    private List<Action> runningActionsLift = new ArrayList<>();
    boolean driverOveride = false;
    boolean clickedX = false;
    MecanumDrive drive;
    Lifters lift;
    Intake intk;
    Follower follower;
    double xMov, yMov = 0;

    @Override
    public void runOpMode(){
//        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        lift = new Lifters(hardwareMap);
        intk = new Intake(hardwareMap);

        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(0,0,0));
        follower.resetIMU();

//        drive.setTeamRed();
//        drive.pauseCamera();
        waitForStart();
        intk.setTrunkPos(190);
        intk.setTwistPos(90);
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


        if(!driverOveride) {
            if (gamepad1.dpad_up) {
                xMov=1;
            } else if (gamepad1.dpad_down) {
                xMov=-1;
            } else if (gamepad1.dpad_left){
                yMov=1;
            } else if (gamepad1.dpad_right){
                yMov=-1;
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
            intk.intakeIn();
        } else if(gamepad2.b) {  //intake out
            intk.intakeOut();
        }else intk.intakeOff();  //turn off when not touching those two buttons

        //lifter control code

        if(gamepad2.dpad_down) {  //go down to specimen wall
            runningActions.add(lift.setVertLifterPos(700, .5));
        }
        else if(gamepad2.dpad_up){ //go up to pole
            runningActions.add(lift.setVertLifterPos(2520, .5));
        }
        else{  //lift using triggers
            if(Math.abs(-gamepad2.right_trigger+gamepad2.left_trigger)>0.01){
                lift.vertLifterR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lift.vertLifterL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lift.setVertLifterPower(-gamepad2.right_trigger+gamepad2.left_trigger);
            }else{
                lift.lifterOverideOff();
            }
        }

        intk.setTwistPower(gamepad2.left_stick_x); //twist using left joystick
        if(gamepad2.left_bumper){  //left bumper = extendo forward
            lift.setHorLifterPower(1);
        } else if(gamepad2.right_bumper) { //right bumper = extendo backward
            lift.setHorLifterPower(-1);
        }
        intk.setTrunkPower(-gamepad2.right_stick_y); //trunk up and down using right joystick
//        if(gamepad2.x && !clickedX){
//            clickedX = true;
//            runningActions.add(
//                    new SequentialAction(
//                    lift.setVertLifterPos(200, .5),
//                             new ParallelAction(
//                                     intk.IntakeIn(),
//                                     lift.setVertLifterPos(100, .5)
//                             )
//                    )
//            );
//        } else if(gamepad1.x) {
//            clickedX = true;
//        } else if(!gamepad1.x) {
//            clickedX = false;
//        }

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
        telemetry.update();

    }
}