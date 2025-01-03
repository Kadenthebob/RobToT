package org.firstinspires.ftc.teamcode.sections;
// RR-specific imports

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Lifters {
    public DcMotor vertLifterR, vertLifterL;
    public ServoImplEx horLifterR, horLifterL;
    double horLiftPos = 0;
    int targetPos = 0;
    boolean lifterWhileOn = false;
    boolean lifterOveride = false;
    public static class Params {
        public int lifterLimitHigh = 4050;
        public int lifterLimitLow = 0;
        public double lifterCorCoef = .0008;
        //divide by two because of the 1:2 gear ratio on the extendo arm
    }
    @Config
    public static class ServoExtendParams{
        public static double EXTEND_OFFSET = 0;

        public static double EXTEND_DEGREE_LIMIT = 90;//200
        public static double EXTEND_DEGREE_MAX = 150;
        public static double EXTEND_POWER_COEFF = 4;
    }
    ServoExtendParams servoExtendParams = new ServoExtendParams();
    Params PARAMS = new Params();
    public Lifters(HardwareMap hardwareMap) {
        vertLifterR = hardwareMap.get(DcMotor.class, "lifterR");
        vertLifterR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vertLifterR.setDirection(DcMotor.Direction.REVERSE);
        vertLifterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertLifterR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        vertLifterL = hardwareMap.get(DcMotor.class, "lifterL");
        vertLifterL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vertLifterL.setDirection(DcMotor.Direction.FORWARD);
        vertLifterL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertLifterL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        horLifterR = hardwareMap.get(ServoImplEx.class,"horLifterR");
        horLifterL = hardwareMap.get(ServoImplEx.class,"horLifterL");
        horLifterR.setDirection(Servo.Direction.FORWARD);
        horLifterL.setDirection(Servo.Direction.REVERSE);
        horLifterR.setPwmRange(new PwmControl.PwmRange(500,2500));
        horLifterL.setPwmRange(new PwmControl.PwmRange(500,2500));

    }

    public class LifterWhile implements Action {
        double rPos,lPos,lifterAvgPos;
        double power = .1;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!lifterOveride) {
                int pos = targetPos;
                rPos = vertLifterR.getCurrentPosition();
                lPos = vertLifterL.getCurrentPosition();
                lifterAvgPos = (rPos + lPos) / 2;
                vertLifterR.setTargetPosition(pos);
                vertLifterL.setTargetPosition(pos);
                vertLifterR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                vertLifterL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                vertLifterR.setPower(power + ((lifterAvgPos - rPos) * PARAMS.lifterCorCoef));
                vertLifterL.setPower(power + ((lifterAvgPos - lPos) * PARAMS.lifterCorCoef));
            }

            return (Math.abs(targetPos-lifterAvgPos) > 15 || lifterWhileOn);
        }
    }

    public Action lifterHold(){
        lifterWhileOn = true;
        return new LifterWhile();
    }

    public Action lifterHoldOff(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double rPos = vertLifterR.getCurrentPosition();
                double lPos = vertLifterL.getCurrentPosition();
                double lifterAvgPos = (rPos + lPos) / 2;

                return Math.abs(targetPos-lifterAvgPos) > 15;
            }
        };
    }

    public Action waitForLifter(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return false;
            }
        };
    }

    public void lifterOverideOff(){
        lifterOveride = false;
    }

    public class SetVertLifterPos implements Action {
        double rPos,lPos,lifterAvgPos;
        int pos = 0;
        double power = 0;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            rPos = vertLifterR.getCurrentPosition();
            lPos = vertLifterL.getCurrentPosition();
            lifterAvgPos = (rPos + lPos) / 2;

            vertLifterR.setPower(power + ((lifterAvgPos - rPos) * PARAMS.lifterCorCoef));
            vertLifterL.setPower(power + ((lifterAvgPos - lPos) * PARAMS.lifterCorCoef));

            if(Math.abs(pos-lifterAvgPos) > 15){
                lifterOveride = true;
                return true;
            }else{
                lifterOveride = false;
                return false;
            }
        }
    }
    //stupid setup sh*t
    public Action setVertLifterPos(int posPer, double powerPer) {

        return new Action(){
            double rPos,lPos,lifterAvgPos;
            int pos = posPer;
            double power = powerPer;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                vertLifterR.setTargetPosition(posPer);
                vertLifterL.setTargetPosition(posPer);
                vertLifterR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                vertLifterL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                
                rPos = vertLifterR.getCurrentPosition();
                lPos = vertLifterL.getCurrentPosition();
                lifterAvgPos = (rPos + lPos) / 2;

                vertLifterR.setPower(power + ((lifterAvgPos - rPos) * PARAMS.lifterCorCoef));
                vertLifterL.setPower(power + ((lifterAvgPos - lPos) * PARAMS.lifterCorCoef));

                if(Math.abs(pos-lifterAvgPos) > 50){
                    lifterOveride = true;
                    return true;
                }else{
                    lifterOveride = false;
                    return false;
                }
            }
        };
    }

    public void setVertLifterPower(double pow){
        double lifterRpower = pow;
        double lifterLpower = pow;
        double rPos = vertLifterR.getCurrentPosition();
        double lPos = vertLifterL.getCurrentPosition();
        double lifterAvgPos = Math.max(Math.min((rPos+lPos)/2,PARAMS.lifterLimitHigh),PARAMS.lifterLimitLow);

        lifterRpower += (lifterAvgPos-rPos)*PARAMS.lifterCorCoef;
        lifterLpower += (lifterAvgPos-lPos)*PARAMS.lifterCorCoef;


        if(PARAMS.lifterLimitHigh>lifterAvgPos && pow>=0){
            vertLifterR.setPower(lifterRpower);
            vertLifterL.setPower(lifterLpower);
        }else if(PARAMS.lifterLimitLow<lifterAvgPos && pow<=0) {
            vertLifterR.setPower(lifterRpower);
            vertLifterL.setPower(lifterLpower);
        }else{
            vertLifterR.setPower(lifterRpower-pow);
            vertLifterL.setPower(lifterLpower-pow);
        }
        targetPos = (int)lifterAvgPos;
        lifterOveride = true;
    }


    public void setHorLifterPower(double pow){
        horLiftPos = ServoExtendParams.EXTEND_DEGREE_MAX*(horLifterL.getPosition())-ServoExtendParams.EXTEND_OFFSET;
        setHorLifterPos(horLiftPos+pow*ServoExtendParams.EXTEND_POWER_COEFF);
        //convert average lift pos to degrees then multiply by coeff
    }

    //stupid setup sh*t
    public void setHorLifterPos(double degree){
        double pos = Math.max(Math.min(degree,servoExtendParams.EXTEND_DEGREE_LIMIT),0); //sets a limit to what you can set the servo position to go to

        pos/=ServoExtendParams.EXTEND_DEGREE_MAX; //converts from degrees(0-360) to servo position(0-1)

        horLifterR.setPosition(pos+ServoExtendParams.EXTEND_OFFSET/servoExtendParams.EXTEND_DEGREE_MAX);
        horLifterL.setPosition(pos+ServoExtendParams.EXTEND_OFFSET/servoExtendParams.EXTEND_DEGREE_MAX);
    }
    public Action SetHorLifterPos(double degree) {
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                setHorLifterPos(degree);
                return false;
            }
        };

    }

}

