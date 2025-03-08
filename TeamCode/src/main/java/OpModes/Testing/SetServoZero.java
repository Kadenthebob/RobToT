package OpModes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import sections.Intake;
import sections.Lifters;


@Autonomous(name = "Set ZERO", group = "Auto Debug")
public final class SetServoZero extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Lifters lift = new Lifters(hardwareMap);
        Intake intk = new Intake(hardwareMap);

        waitForStart();
        intk.setClawPos(0);
        intk.setElbowPos(0);
        intk.diffyReset();
        intk.setClaw
        while(opModeIsActive()){

        }

    }

}
