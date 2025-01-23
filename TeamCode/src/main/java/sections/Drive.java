package sections;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.util.PIDFController;

import pedroPathing.constants.AutoGrabConstants;



public class Drive extends Follower{
    public Follower follower;
    PIDFController autoXMovePIDF;
    PIDFController autoYMovePIDF;

    boolean autoGrabbing = false;
    boolean looping = false;

    VoltageSensor voltageSensor;
    TouchSensor touchFront;

    AutoGrabConstants FollowerConstansts = new AutoGrabConstants();
    public Drive(HardwareMap hardwareMap){
        super(hardwareMap);
        autoYMovePIDF = new PIDFController(AutoGrabConstants.AutoMoveDrivePIDFCoefficients);
        autoXMovePIDF = new PIDFController(AutoGrabConstants.AutoMoveDrivePIDFCoefficients);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        touchFront = hardwareMap.get(TouchSensor.class, "touchFront");
    }

    public Action FollowPath(Path path, boolean holdEnd){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                followPath(path, holdEnd);
                return false;
            }
        };
    }
    public Action FollowPath(PathChain pathChain){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                followPath(pathChain, false);
                return false;
            }
        };
    }
    public Action FollowPath(PathChain pathChain, boolean holdEnd){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                followPath(pathChain, holdEnd);
                return false;
            }
        };
    }

    public Action FollowPath(Path path){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                followPath(path, true);
                return false;
            }
        };
    }

    public Action waitForX(double x) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return (Math.abs(poseUpdater.getPose().getX()-x)>=Math.abs(.03*poseUpdater.getPose().getX()));
            }
        };
    }

    public Action waitForY(double y) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return (Math.abs(poseUpdater.getPose().getY()-y)>=Math.abs(.015*poseUpdater.getPose().getY()));
            }
        };
    }

    public Action waitForHeading(double yaw) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return (Math.abs(poseUpdater.getPose().getHeading()-yaw)>=Math.abs(.015*poseUpdater.getPose().getHeading()));
            }
        };
    }

    public Action waitForPoint(double x, double y) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return (Math.abs(poseUpdater.getPose().getY()-y)>=Math.abs(.015*poseUpdater.getPose().getY())&&Math.abs(poseUpdater.getPose().getX()-x)>=Math.abs(.015*poseUpdater.getPose().getX()));
            }
        };
    }

    public Action waitForPoint(Pose pos) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return (Math.abs(poseUpdater.getPose().getY()-pos.getY())>=Math.abs(.015*poseUpdater.getPose().getY())&&Math.abs(poseUpdater.getPose().getX()-pos.getX())>=Math.abs(.015*poseUpdater.getPose().getX()));
            }
        };
    }

    public Action waitForPose(Pose pos) {
        return new Action() {
            ElapsedTime time;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if(getVelocity().getMagnitude()>.75){
                    time.reset();
                }
                if(time.time()>1.5){
                    return false;
                }
                return (!pos.roughlyEquals(getPose(),.6));
            }
        };
    }
    Pose start;
    public Action PostAutoMove() {
        return new Action() {
            ElapsedTime time = new ElapsedTime();
            Boolean wait = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if(!wait){
                    time.reset();
                    followPath(
                            pathBuilder()
                                    .addPath(
                                            new BezierLine(
                                                    new Point(getPose()),
                                                    new Point(start)
                                            )
                                    )
                                    .setConstantHeadingInterpolation(start.getHeading())
                                    .build()
                            ,true);
                    wait = true;
                }

                if(start.roughlyEquals(getPose(),.2)||time.time()>2.5)
                    return false;
                return true;
            }
        };
    }
    public Action AutoMove(Camera cam) {
        return new Action() {
            double rotation, targetYaw, cam2inch;
            double x;
            double y;
            Pose endPoint;
            Pose endPointSV;
            boolean driving = false;
            boolean looping = false;
            boolean wait = false;
            ElapsedTime time = new ElapsedTime();
            ElapsedTime driveReset = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                x = cam.targetAdjX();
                y = cam.targetAdjY();
                if ((time.time() < 5 && (x == -1 && y == -1) || !wait) && !driving) {
                    cam.resetTarget();
                    start = getPose();
                    breakFollowing();
                    packet.put("obj x", x);
                    packet.put("obj y", y);
                    packet.put("w1", cam.getWidth());
                    packet.put("w2", cam.getHeight());
                    wait = true;
                    return true;

                } else if ((x == -1 && y == -1) && !driving) {
                    //if no obj detected
                    return false;
                } else if (!driving) {
                    time.reset();
                    start = getPose();

                    double xChange = (cam.getTargetX() - x) * cam.getCam2Inch();
                    double yChange = (cam.getTargetY() - y) * cam.getCam2Inch();

                    endPoint = MathFunctions.rotatePose(new Pose(yChange + AutoGrabConstants.autoGrabOffX, xChange+AutoGrabConstants.autoGrabOffY, 0), start.getHeading(), true);
                    endPoint.add(start);
                    followPath(
                            pathBuilder().addPath(
                                            new BezierLine(
                                                    new Point(start),
                                                    new Point(endPoint)
                                            )
                                    )
                                    .setConstantHeadingInterpolation(start.getHeading())
                                    .build()
                            , true);
                    driving = true;
                    endPointSV = endPoint.copy();
                    driveReset.reset();
                    return true;

                } else {
                    cam.resetTarget();
                    if(x!=-1 && false) {
                        double xChange = (cam.getTargetX() - x) * cam.getCam2Inch();
                        double yChange = (cam.getTargetY() - y) * cam.getCam2Inch();

                        endPoint = MathFunctions.rotatePose(new Pose(yChange + AutoGrabConstants.autoGrabOffX, xChange + AutoGrabConstants.autoGrabOffY, 0), start.getHeading(), true);
                        endPoint.add(start);

                        if (driveReset.time() > .1) {
                            followPath(
                                    pathBuilder().addPath(
                                                    new BezierLine(
                                                            new Point(start),
                                                            new Point(endPoint)
                                                    )
                                            )
                                            .setConstantHeadingInterpolation(start.getHeading())
                                            .build()
                                    , true);
                            endPointSV = endPoint.copy();
                            driveReset.reset();
                        }
                    }
                    if (endPoint.roughlyEquals(getPose(), .5) || time.time() > 2.5) {
                        return false;
                    }
                    packet.put("obj x", x);
                    packet.put("obj y", y);
                    packet.put("time", time.time());
                    return true;
                }
            }
        };
    }

    public Action AutoMoveLoop(Camera cam,Intake intk) {
        return new Action() {
            double rotation, targetYaw, cam2inch;
            double x;
            double y;
            double tarHeading;
            Pose endPoint;
            boolean driving = false;
            boolean looping = false;
            boolean wait = false;
            ElapsedTime time = new ElapsedTime();
            ElapsedTime moveTime = new ElapsedTime();
            ElapsedTime pathTimeout = new ElapsedTime();  // Added timeout for path following

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                x = cam.targetAdjX();
                y = cam.targetAdjY();
                // Fixed parentheses for correct logic evaluation

                if(!wait){
                    autoXMovePIDF.reset();
                    autoYMovePIDF.reset();
                    time.reset();
                    moveTime.reset();
                    cam.resetTarget();
                    start = getPose();
                    breakFollowing();
                    wait = true;
                    looping = true;
                    autoGrabbing = true;
                    targetYaw = start.getHeading();
                }
                if(time.time()>5){
                    return false;
                }
//
                if ((Math.abs(cam.getTargetX() - x) > 10 ||
                        Math.abs(cam.getTargetY() - y) > 10) && !driving && moveTime.time()<2.5) {
                    startTeleopDrive();

                    // Calculate movement vectors
                    autoXMovePIDF.updateError(cam.getTargetX()-cam.getObjX());
                    autoYMovePIDF.updateError(cam.getTargetY()-cam.getObjY());

                    packet.put("obj x", x);
                    packet.put("obj y", y);

                    double headingChange = cam.angleCor(1 * (targetYaw - poseUpdater.getPose().getHeading()));
                    if((x == -1 && y == -1)){
                        setTeleOpMovementVectors(0,0,headingChange);
                        return true;
                    }
                    // Single call to setTeleOpMovementVectors
                    setTeleOpMovementVectors(MathFunctions.clamp(12*(autoYMovePIDF.runPIDF()/voltageSensor.getVoltage()),-AutoGrabConstants.autoGrabMaxPower,AutoGrabConstants.autoGrabMaxPower), MathFunctions.clamp(12*(autoXMovePIDF.runPIDF()/voltageSensor.getVoltage()),-AutoGrabConstants.autoGrabMaxPower,AutoGrabConstants.autoGrabMaxPower), headingChange);
                    return true;

                } else if(!driving && looping) {
                    // Set teleopDrive before path construction
                    if((x == -1 && y == -1)){
                        return false;
                    }

                    start = getPose();
                    double xChange = (cam.getTargetX() - x) * cam.getCam2Inch();
                    double yChange = (cam.getTargetY() - y) * cam.getCam2Inch();

                    endPoint = MathFunctions.rotatePose(new Pose(yChange + AutoGrabConstants.autoGrabOffX, xChange+AutoGrabConstants.autoGrabOffY, 0), start.getHeading(), true);
                    endPoint.add(start);

                    cam.setSelected();

                    intk.setTwistMatchObjAngle(cam);
                    intk.setClawAutoOpen(cam);

                    followPath(
                            pathBuilder().addPath(
                                            new BezierLine(
                                                    new Point(start),
                                                    new Point(endPoint)
                                            )
                                    )
                                    .setConstantHeadingInterpolation(start.getHeading())
                                    .build()
                            , true);

                    driving = true;
                    pathTimeout.reset();  // Start timeout timer
                    return true;

                    // Added timeout condition (10 seconds) for path following
                } else if((endPoint.roughlyEquals(getPose(), .7) || pathTimeout.time() > 1.5)&&driving) {
                    autoGrabbing = false;
                    return false;
                }
                return true;
            }
        };
    }

    public Action AutoMoveLoopOnly(Camera cam,Intake intk) {
        return new Action() {
            double rotation, targetYaw, cam2inch;
            double x;
            double y;
            double tarHeading;
            Pose endPoint;
            boolean driving = false;
            boolean looping = false;
            boolean wait = false;
            ElapsedTime time = new ElapsedTime();
            ElapsedTime moveTime = new ElapsedTime();
            ElapsedTime pathTimeout = new ElapsedTime();  // Added timeout for path following

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                x = cam.targetAdjX();
                y = cam.targetAdjY();
                // Fixed parentheses for correct logic evaluation

                if(!wait){
                    autoXMovePIDF.reset();
                    autoYMovePIDF.reset();
                    time.reset();
                    moveTime.reset();
                    cam.resetTarget();
                    start = getPose();
                    breakFollowing();
                    wait = true;
                    looping = true;
                    autoGrabbing = true;
                    targetYaw = start.getHeading();
                    startTeleopDrive();
                }
                if(time.time()>5){
                    return false;
                }
//
                if ((Math.abs(cam.getTargetX() - x) > AutoGrabConstants.objLoopDistance ||
                        Math.abs(cam.getTargetY() - y) > AutoGrabConstants.objLoopDistance) && !driving && moveTime.time()<2.5) {


                    // Calculate movement vectors
                    autoXMovePIDF.updateError(320-cam.getObjX());
                    autoYMovePIDF.updateError(180-cam.getObjY());

                    packet.put("obj x", x);
                    packet.put("obj y", y);

                    double headingChange = cam.angleCor(1 * (targetYaw - poseUpdater.getPose().getHeading()));
                    if((x == -1 && y == -1)){
                        setTeleOpMovementVectors(0,0,headingChange);
                        return true;
                    }
                    // Single call to setTeleOpMovementVectors
                    setTeleOpMovementVectors(MathFunctions.clamp(12*(autoYMovePIDF.runPIDF()/voltageSensor.getVoltage()),-AutoGrabConstants.autoGrabMaxPower,AutoGrabConstants.autoGrabMaxPower), MathFunctions.clamp(12*(autoXMovePIDF.runPIDF()/voltageSensor.getVoltage()),-AutoGrabConstants.autoGrabMaxPower,AutoGrabConstants.autoGrabMaxPower), headingChange);
                    return true;

                }
                endPoint = getPose();
                followPath(
                        pathBuilder().addPath(
                                        new BezierLine(
                                                new Point(endPoint),
                                                new Point(endPoint)
                                        )
                                )
                                .setConstantHeadingInterpolation(endPoint.getHeading())
                                .build()
                        , true);
                return false;
            }
        };
    }

    public Action WaitForDetect(Camera cam){
        return new Action() {
            double rotation,targetYaw,cam2inch;
            double x;
            double y;
            Pose endPoint;
            boolean driving = false;
            boolean wait = false;
            ElapsedTime time = new ElapsedTime();
            @Override
            public boolean run (@NonNull TelemetryPacket packet) {
                x = cam.getObjX();
                y = cam.getObjY();
                rotation = cam.getObjRot();
                if (((time.time() < 5 && (x == -1 && y == -1) || !wait) && !driving)||AutoGrabConstants.autoWait) {
//                    cam.resetTarget();
                    start = getPose();
                    breakFollowing();
                    double xChange = (cam.getTargetX() - x) * cam.getCam2Inch();
                    double yChange = (cam.getTargetY() - y) * cam.getCam2Inch();
                    packet.put("obj x", Math.floor(x));
                    packet.put("obj y", Math.floor(y));
                    packet.put("obj x adj", Math.floor(x));
                    packet.put("obj y adj", Math.floor(cam.targetAdjY()));
                    packet.put("x Change", Math.floor((xChange+AutoGrabConstants.autoGrabOffY)*100)/100);
                    packet.put("y Change", Math.floor((yChange+AutoGrabConstants.autoGrabOffX)*100)/100);
                    packet.put("x dif", Math.floor(cam.getTargetDifX()*100)/100);
                    packet.put("y dif", Math.floor(cam.getTargetDifY()*100)/100);
                    wait = true;
                    return true;

                }
                cam.setSelected();
                if(AutoGrabConstants.autoWait){
                    return true;
                }
                return false;
            }
        };
    }

    public Action AutoGrab(Camera cam, Intake intk, Lifters lift){
        return AutoGrab(cam, intk, lift, false);
    }

    public Action AutoGrab(Camera cam, Intake intk, Lifters lift, Boolean teleOp){
        Action tele = new SleepAction(0);
        Action wait = new SleepAction(0);
        if(teleOp){
            tele = new InstantAction(()->startTeleopDrive());
        }
        if(AutoGrabConstants.autoWait){
            wait = WaitForDetect(cam);
        }
        return new SequentialAction(
                lift.setVertLifterPos(700,1),
                intk.SetElbowPos(15),
                intk.autoOverideOn(),
                intk.SetClawAutoOpen(cam),
                intk.SetTwistMatchObjAngle(cam),
                wait,
                AutoMoveLoopOnly(cam,intk),
                new SleepAction(.1),
                new ParallelAction(
//                        AutoMoveLoop(cam, intk)
                        AutoMove(cam),
                        lift.setVertLifterPos(250,1)
                ),
                new SleepAction(.5),
                lift.setVertLifterZero(1),
                new SleepAction(.075),
                intk.SetClawAutoClose(cam),
                intk.autoOverideOff(),
                tele
        );
    }
    public Action goToPose(Pose pos){
        return FollowPath(pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(getPose()),
                                new Point(pos)
                        )
                )
                .setLinearHeadingInterpolation(getPose().getHeading(), pos.getHeading())
                .build());
    }

    public boolean getTeleOpOveride(Camera cam){
        if(autoGrabbing&&cam.getObjX()!=-1){
            return true;
        } else return false;
    }
    public void touchReset(double posFront){
        if(touchFront.isPressed()){
            setPose(new Pose(posFront,getPose().getY(),getPose().getHeading()));
        }
    }
    public Action touchSensorLoop(double x){
        return new Action(){
            @Override
            public boolean run (@NonNull TelemetryPacket packet) {
                touchReset(x);
                return true;
            }
        };
    }
    public Action Update(){
        return new Action(){
            boolean start = true;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if(start == true){
                    looping = true;
                    start = false;
                    update();
                    return true;
                }
                if(looping == true) {
                    update();
                    return true;
                }
                return false;
            }
        };
    }
    public Action Update(MultipleTelemetry tele){
        return new Action(){
            boolean start = true;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if(start == true){
                    looping = true;
                    start = false;
                    update();
                    return true;
                }
                if(looping == true) {
                    update();
                    telemetryDebug(tele);
                    return true;
                }
                return false;
            }
        };
    }

    public Action StopUpdate(){
        return new Action(){
            boolean start = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                looping = false;
                return false;
            }
        };
    }
}
