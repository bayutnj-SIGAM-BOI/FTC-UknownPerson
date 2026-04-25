package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Objects;

@Config
public final class PinpointLocalizer implements Localizer {

    public static class Params {
        public double parXTicks = 0.0;   // parallel encoder (forward → X)
        public double perpYTicks = 0.0;  // perpendicular encoder (strafe → Y)
    }

    public static Params PARAMS = new Params();

    public final GoBildaPinpointDriver driver;
    public final GoBildaPinpointDriver.EncoderDirection initialParDirection;
    public final GoBildaPinpointDriver.EncoderDirection initialPerpDirection;

    private Pose2d txWorldPinpoint;
    private Pose2d txPinpointRobot = new Pose2d(0, 0, 0);

    public PinpointLocalizer(HardwareMap hardwareMap, double inPerTick, Pose2d initialPose) {

        driver = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // inch → mm
        double mmPerTick = inPerTick * 25.4;

        // resolution
        driver.setEncoderResolution(1 / mmPerTick, DistanceUnit.MM);

        // ✅ FIX: offset sesuai axis (X forward, Y strafe)
        driver.setOffsets(
                mmPerTick * PARAMS.parXTicks,
                mmPerTick * PARAMS.perpYTicks,
                DistanceUnit.MM
        );

        // arah encoder (ubah kalau kebalik)
        initialParDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        initialPerpDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;

        driver.setEncoderDirections(initialParDirection, initialPerpDirection);

        driver.resetPosAndIMU();

        txWorldPinpoint = initialPose;
    }

    @Override
    public void setPose(Pose2d pose) {
        txWorldPinpoint = pose.times(txPinpointRobot.inverse());
    }

    @Override
    public Pose2d getPose() {
        return txWorldPinpoint.times(txPinpointRobot);
    }

    @Override
    public PoseVelocity2d update() {
        driver.update();

        if (Objects.requireNonNull(driver.getDeviceStatus()) ==
                GoBildaPinpointDriver.DeviceStatus.READY) {

            // posisi dari Pinpoint
            txPinpointRobot = new Pose2d(
                    driver.getPosX(DistanceUnit.INCH),
                    driver.getPosY(DistanceUnit.INCH),
                    driver.getHeading(UnnormalizedAngleUnit.RADIANS)
            );

            // velocity world frame
            Vector2d worldVelocity = new Vector2d(
                    driver.getVelX(DistanceUnit.INCH),
                    driver.getVelY(DistanceUnit.INCH)
            );

            // convert ke robot frame
            Vector2d robotVelocity =
                    Rotation2d.fromDouble(-txPinpointRobot.heading.log())
                            .times(worldVelocity);

            return new PoseVelocity2d(
                    robotVelocity,
                    driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)
            );
        }

        return new PoseVelocity2d(new Vector2d(0, 0), 0);
    }
}