package OpModes.TeamSettings;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import sections.Camera;


@TeleOp(name = "SetTeamRed", group = "Cam Settings")
public final class SetTeamRed extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {
        Camera.CamParams.redTeam = true;
    }

}
