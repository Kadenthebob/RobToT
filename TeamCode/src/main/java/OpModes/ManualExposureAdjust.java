package OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import sections.*;


@TeleOp(name = "Manual Exposure Control", group = "Auto Testing")
public final class ManualExposureAdjust extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {

        Lifters lift = new Lifters(hardwareMap);
        Intake intk = new Intake(hardwareMap);
        Camera cam = new Camera(hardwareMap,true);
        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.a){
                cam.adjustExposure((long)(-gamepad1.right_stick_y*10));
            }

            telemetry.addData("exposure",cam.getExposure());
            telemetry.update();
        }
    }

}
