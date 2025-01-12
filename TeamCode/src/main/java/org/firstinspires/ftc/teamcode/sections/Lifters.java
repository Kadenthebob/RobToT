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
import com.qualcomm.robotcore.hardware.TouchSensor;;

public class Lifters {
    public DcMotor vertLifterR, vertLifterL;
    public ServoImplEx horLifterR, horLifterL;
    public TouchSensor magR,magL;
    double horLiftPos = 0;
    int targetPosL = 0;
    int targetPosR = 0;
    boolean lifterWhileOn = false;
    boolean lifterOveride = false;
    boolean rMagReset = false;
    boolean lMagReset = false;

    @Config
    public static class liftParams {
        public int lifterLimitHigh = 4050;
        public int lifterLimitLow = 0;
        public double lifterCorCof = .0008;
        public static boolean doLiftEncoderEqualiser = false;
        public double getLifterCorCoef(){
            if(doLiftEncoderEqualiser){
                return lifterCorCof;
            } else return 0;
        }
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
    liftParams PARAMS = new liftParams();
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

        magR = hardwareMap.get(TouchSensor.class,"magR");
        magL = hardwareMap.get(TouchSensor.class,"magL");

    }

    public class LifterWhile implements Action {
        double rPos,lPos,lifterAvgPos;
        double power = .1;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!lifterOveride) {
                hallCheck();
                rPos = vertLifterR.getCurrentPosition();
                lPos = vertLifterL.getCurrentPosition();
                lifterAvgPos = (rPos + lPos) / 2;
                vertLifterR.setTargetPosition(targetPosR);
                vertLifterL.setTargetPosition(targetPosL);
                vertLifterR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                vertLifterL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                vertLifterR.setPower(power + ((lifterAvgPos - rPos) * PARAMS.getLifterCorCoef()));
                vertLifterL.setPower(power + ((lifterAvgPos - lPos) * PARAMS.getLifterCorCoef()));
            }

            return ((Math.abs(targetPosL-lPos) > 15 && Math.abs(targetPosR-rPos) > 15) || lifterWhileOn);
        }
    }

    public Action lifterHold(){
        lifterWhileOn = true;
        return new LifterWhile();
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

    //stupid setup sh*t
    public Action setVertLifterPos(int posPer, double powerPer) {

        return new Action(){
            double rPos,lPos,lifterAvgPos;
            int pos = posPer;
            double power = powerPer;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hallCheck();
                lifterOveride = true;
                vertLifterR.setTargetPosition(posPer);
                vertLifterL.setTargetPosition(posPer);
                vertLifterR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                vertLifterL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                
                rPos = vertLifterR.getCurrentPosition();
                lPos = vertLifterL.getCurrentPosition();
                lifterAvgPos = (rPos + lPos) / 2;

                vertLifterR.setPower(power + ((lifterAvgPos - rPos) * PARAMS.getLifterCorCoef()));
                vertLifterL.setPower(power + ((lifterAvgPos - lPos) * PARAMS.getLifterCorCoef()));

                if(Math.abs(pos-lifterAvgPos) > 50){
                    return true;
                }else{
                    targetPosR = vertLifterR.getCurrentPosition();
                    targetPosL = vertLifterL.getCurrentPosition();
                    lifterOveride = false;
                    return false;
                }
            }
        };
    }

    public Action setVertLifterZero(double powerPer) {

        return new Action(){

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                vertLifterR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                vertLifterL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lifterOveride = true;
                hallCheck();
                if(!hallZeroCheck()) {
                    vertLifterR.setPower(-Math.abs(powerPer));
                    vertLifterL.setPower(-Math.abs(powerPer));
                    return true;
                } else{
                    lifterOveride = false;
                    targetPosR = vertLifterR.getCurrentPosition();
                    targetPosL = vertLifterL.getCurrentPosition();
                    vertLifterR.setPower(0);
                    vertLifterL.setPower(0);
                    return false;
                }

            }
        };
    }

    public void setVertLifterPower(double pow){
        if(!lifterOveride) {
            hallCheck();



            double lifterRpower = pow;
            double lifterLpower = pow;
            double rPos = vertLifterR.getCurrentPosition();
            double lPos = vertLifterL.getCurrentPosition();
            double lifterAvgPos = Math.max(Math.min((rPos+lPos)/2,PARAMS.lifterLimitHigh),PARAMS.lifterLimitLow);

            lifterRpower += (lifterAvgPos-rPos)*PARAMS.getLifterCorCoef();
            lifterLpower += (lifterAvgPos-lPos)*PARAMS.getLifterCorCoef();

            if (PARAMS.lifterLimitHigh > lifterAvgPos && pow >= 0) {
                vertLifterR.setTargetPosition(PARAMS.lifterLimitHigh);
                vertLifterL.setTargetPosition(PARAMS.lifterLimitHigh);
                vertLifterR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                vertLifterL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                vertLifterR.setPower(lifterRpower);
                vertLifterL.setPower(lifterLpower);
            } else if (!hallZeroCheck() && pow <= 0) {
                vertLifterR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                vertLifterL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                vertLifterR.setPower(lifterRpower);
                vertLifterL.setPower(lifterLpower);
            } else {
                vertLifterR.setPower(lifterRpower - pow);
                vertLifterL.setPower(lifterLpower - pow);
            }
            targetPosR = vertLifterR.getCurrentPosition();
            targetPosL = vertLifterL.getCurrentPosition();
        }
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
    public void hallCheck(){
        if(magR.isPressed()&&!rMagReset){
            vertLifterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rMagReset = true;
        }else if (!magR.isPressed()) rMagReset = false;

        if(magL.isPressed()&&!lMagReset){
            vertLifterL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lMagReset = true;
        }else if (!magL.isPressed()) lMagReset = false;
    }

    public boolean hallZeroCheck(){
        return (magR.isPressed() || magL.isPressed());
    }

    public void resetEncoders(){
        vertLifterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertLifterL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}

