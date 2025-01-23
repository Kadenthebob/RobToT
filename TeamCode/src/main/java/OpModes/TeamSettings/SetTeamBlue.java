package OpModes.TeamSettings;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import sections.Camera;


@TeleOp(name = "SetTeamBlue", group = "Cam Settings")
public final class SetTeamBlue extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {
        Camera.CamParams.redTeam = false;
    }

}
