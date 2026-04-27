import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

@TeleOp(name = "Pinpoint Direction Check")
public class PinpointDirection extends LinearOpMode {
    @Override
    public void runOpMode() {
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();

        waitForStart();

        while (opModeIsActive()) {
            pinpoint.update();
            telemetry.addData("posX (harusnya + kalau maju)", pinpoint.getPosX(DistanceUnit.INCH));
            telemetry.addData("posY (harusnya + kalau geser kanan)", pinpoint.getPosY(DistanceUnit.INCH));
            telemetry.addData("heading (harusnya + kalau putar CCW)", Math.toDegrees(pinpoint.getHeading(UnnormalizedAngleUnit.RADIANS)));
            telemetry.update();
        }
    }
}