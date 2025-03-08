package OpModes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import sections.Intake;
import sections.Lifters;
import sections.Outake;


@Autonomous(name = "Set ZERO", group = "Auto Debug")
public final class SetServoZero extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Lifters lift = new Lifters(hardwareMap);
        Intake intk = new Intake(hardwareMap);
        Outake out = new Outake(hardwareMap);

        waitForStart();
        out.setClawClose();
        out.setElbowPos(0);
        out.diffyReset();

//        intk.setClawClose();
//        intk.setElbowPos(0);
//        intk.diffyReset();
        while(opModeIsActive()){

        }

    }

}
