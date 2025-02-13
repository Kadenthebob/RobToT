package OpModes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.concurrent.TimeUnit;

import sections.*;


@TeleOp(name = "Camera Control", group = "Auto Testing")
public final class ManualExposureAdjust extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {
        Camera cam = new Camera(hardwareMap,true);
        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.a){
                cam.adjustExposure((long)(-gamepad1.right_stick_y*10));
            }

            telemetry.addData("exposure",cam.getExposure());
            telemetry.addData("exposure max",cam.cam.getExposureControl().getMaxExposure(TimeUnit.MICROSECONDS));

            telemetry.addData("gain",cam.cam.getGainControl().getGain());
            telemetry.addData("gain min",cam.cam.getGainControl().getMinGain());
            telemetry.addData("gain max",cam.cam.getGainControl().getMaxGain());

            telemetry.addData("focus",cam.cam.getFocusControl().getFocusLength());
            telemetry.addData("focus min",cam.cam.getFocusControl().getMinFocusLength());
            telemetry.addData("focus max",cam.cam.getFocusControl().getMaxFocusLength());
            telemetry.update();
        }
    }

}
