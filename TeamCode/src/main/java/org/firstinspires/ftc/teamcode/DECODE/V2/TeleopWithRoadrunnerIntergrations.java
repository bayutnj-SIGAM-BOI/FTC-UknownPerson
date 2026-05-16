    package org.firstinspires.ftc.teamcode.DECODE.V2;

    import com.acmerobotics.dashboard.config.Config;
    import com.acmerobotics.roadrunner.Pose2d;
    import com.acmerobotics.roadrunner.PoseVelocity2d;
    import com.acmerobotics.roadrunner.Vector2d;
    import com.qualcomm.robotcore.eventloop.opmode.OpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

    import org.firstinspires.ftc.teamcode.DECODE.PIDControl;
    import org.firstinspires.ftc.teamcode.DECODE.RobotStatic;
    import org.firstinspires.ftc.teamcode.MecanumDrive;

    @Config
    @TeleOp(name = "V2 Mecanum")
    public class  TeleopWithRoadrunnerIntergrations extends OpMode {
        private  MecanumDrive drive;
        private PIDControl pidControl;
        private final double SNAP_90 = Math.toRadians(90), SNAP_0 = Math.toRadians(0);
        private Pose2d snapTarget = RobotStatic.blueAimingTarget;
        private final Pose2d StartPose = PosesStorage.currentPose;
        private boolean isBlueAiming = true, isRedAiming = false;
        private boolean autoheading = false;
        double targetAngle;

        @Override
        public void init() {
            drive = new MecanumDrive(hardwareMap, StartPose);
            pidControl = new PIDControl(RobotStatic.HeadingKp, RobotStatic.HeadingKi, RobotStatic.HeadingKd);
        }

        @Override
        public void init_loop() {
            telemetry.addLine("BLUE ALLIANCE (DPAD_LEFT) | RED ALLIANCE (DPAD_RIGHT)");
            if (gamepad1.dpad_left) {
                snapTarget = RobotStatic.blueAimingTarget;
                isBlueAiming = true;
                isRedAiming = false;
                telemetry.addData("Alliance", "BLUE");
            }
            if (gamepad1.dpad_right) {
                snapTarget = RobotStatic.redAimingTarget;
                isRedAiming = true;
                isBlueAiming = false;
                telemetry.addData("Alliance", "RED");
            }
            telemetry.addData("Selected", isBlueAiming ?"BLUE" : "RED");
            telemetry.update();
        }

        @Override
        public void loop() {
            drive.localizer.update();
            Pose2d pos = drive.localizer.getPose();
            double xPose = pos.position.x;
            double yPose = pos.position.y;
            double hPose = pos.heading.toDouble();
            double disTarget = Math.hypot(xPose - snapTarget.position.x, yPose - snapTarget.position.y);

            double slowMultiply = gamepad1.left_bumper ? 0.6 : 1.0;
            double forward = -gamepad1.left_stick_y * slowMultiply;
            double strafe = -gamepad1.left_stick_x * slowMultiply;
            double rotate = -gamepad1.right_stick_x * slowMultiply;

            double cos = Math.cos(hPose);
            double sin = Math.sin(hPose);
            double rotX = forward * cos - strafe * sin;
            double rotY = forward * sin + strafe * cos;

            if (gamepad1.dpad_up) {targetAngle = SNAP_0; autoheading = true;}
            else if (gamepad1.dpad_down) { targetAngle = SNAP_90; autoheading = true;}
            if (gamepad1.right_bumper) {targetAngle = Math.atan2(snapTarget.position.y - yPose, snapTarget.position.x - xPose);autoheading = true;}

            if (autoheading) {
                double error = targetAngle - hPose;

                error = pidControl.angleWrapRadians(error);

                if (Math.abs(error) < Math.toRadians(1)) {
                    autoheading = false;
                }
                rotate = pidControl.calculateRadians(targetAngle, hPose);
            }
            if (Math.abs(-gamepad1.right_stick_x) > 0.5) {autoheading = false;}
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(rotX, rotY), rotate));

//           ================= TELEMETRY =================
            telemetry.addLine("ROBOT POSITION");
            telemetry.addData("START POSE", StartPose);
            telemetry.addData("xPose", xPose);
            telemetry.addData("yPose", yPose);
            telemetry.addData("HPose", Math.toDegrees(hPose));

            telemetry.addLine("ODOMETRY SNAP TARGET");
            telemetry.addData("SNAPPED TARGET", isBlueAiming ? "BLUE ALLIANCE" : "RED ALLIANCE");
            telemetry.addData("SNAP ANGLE", Math.toDegrees(targetAngle));
            telemetry.addData("Snap Goal", Math.toDegrees(Math.atan2(snapTarget.position.y - yPose, snapTarget.position.x - xPose)));
            telemetry.update();
        }
    }
