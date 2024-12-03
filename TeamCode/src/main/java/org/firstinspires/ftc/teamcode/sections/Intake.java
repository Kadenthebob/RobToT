package org.firstinspires.ftc.teamcode.sections;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    public static class Params {
        double intakeSpeed = 1;
        public double trunkMaxDegree = 190;//200
        public double trunkServoMaxTurn = 360*5-180;
        public double trunkPowerCoeff = 2;

        public double twistMaxDegree = 180;//200
        public double twistServoMaxTurn = 360*5-180;
        public double twistPowerCoeff = 2;

    }
    double trunkPos = 0;
    double twistPos = 0;
    Params PARAMS = new Params();
    CRServo intakeR, intakeL;
    public Servo trunkR, trunkL, twist;
    public Intake(HardwareMap hardwareMap){
        intakeR = hardwareMap.get(CRServo.class,"intakeR");
        intakeL = hardwareMap.get(CRServo.class,"intakeL");
        intakeR.setDirection(CRServo.Direction.FORWARD);
        intakeL.setDirection(CRServo.Direction.REVERSE);

        trunkR = hardwareMap.get(Servo.class,"trunkR");
        trunkL = hardwareMap.get(Servo.class,"trunkL");
        twist = hardwareMap.get(Servo.class,"twist");
        trunkR.setDirection(Servo.Direction.FORWARD);
        trunkL.setDirection(Servo.Direction.REVERSE);
        twist.setDirection(Servo.Direction.REVERSE);
    }
    public void intakeIn(){
        intakeR.setPower(PARAMS.intakeSpeed);
        intakeL.setPower(PARAMS.intakeSpeed);
    }

    public void intakeOut(){
        intakeR.setPower(-PARAMS.intakeSpeed);
        intakeL.setPower(-PARAMS.intakeSpeed);
    }

    public void intakeOff(){
        intakeR.setPower(0);
        intakeL.setPower(0);
    }

    //Auto Actions
    public Action IntakeIn(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeIn();
                return false;
            }
        };
    }
    public Action IntakeOut(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeOut();
                return false;
            }
        };
    }
    public Action IntakeOff(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeOff();
                return false;
            }
        };
    }

    public void setTrunkPos(double degree){
        double pos = Math.max(Math.min(degree,PARAMS.trunkMaxDegree),0); //sets a limit to what you can set the servo position to go to

        pos/=PARAMS.trunkServoMaxTurn; //converts from degrees(0-360) to servo position(0-1)

        trunkR.setPosition(pos+(180/PARAMS.trunkServoMaxTurn));
        trunkL.setPosition(pos+(180/PARAMS.trunkServoMaxTurn));
    }

    public void setTrunkPower(double pow){
        trunkPos = PARAMS.trunkServoMaxTurn*((trunkR.getPosition() + trunkL.getPosition())/2)-180;
        setTrunkPos(trunkPos + pow*PARAMS.trunkPowerCoeff);
    }

    public Action SetTrunkPos(double degree){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                setTrunkPos(degree);
                return false;
            }
        };
    }

    public void setTwistPos(double degree){
        double pos = Math.max(Math.min(degree,PARAMS.twistMaxDegree),0); //sets a limit to what you can set the servo position to go to

        pos/=PARAMS.twistServoMaxTurn; //converts from degrees(0-360) to servo position(0-1)

        twist.setPosition(pos);
    }

    public void setTwistPower(double pow){
        twistPos = twist.getPosition()*PARAMS.twistServoMaxTurn;
        setTwistPos(twistPos + pow*PARAMS.twistPowerCoeff);
    }

    public Action SetTwistPos(double degree){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                setTwistPos(degree);
                return false;
            }
        };
    }

}
