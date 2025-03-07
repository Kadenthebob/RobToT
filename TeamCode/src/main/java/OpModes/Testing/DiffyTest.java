package OpModes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import sections.Camera;
import sections.Intake;


@Autonomous(name = "Diffy Test", group = "Auto Debug")
public final class DiffyTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Intake intk = new Intake(hardwareMap);
        Camera cam = new Camera(hardwareMap,false);
        waitForStart();
        boolean holdingA = false;
        while(opModeIsActive()){
            if(gamepad1.a&&!holdingA) {
                holdingA = true;
                intk.setDiffyTwistPos(30);
                sleep(1000);
                intk.setDiffyTwistPos(0);

            } else if(!gamepad1.a){
                holdingA = false;
            }
        }
    }

}
