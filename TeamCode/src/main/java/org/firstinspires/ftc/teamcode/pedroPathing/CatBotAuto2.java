package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.telemetryM;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;
import java.util.function.Supplier;
import com.pedropathing.ftc.FTCCoordinates;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Configurable
@Autonomous(name = "CatBot Auto2", group = "Autonomous")
public class CatBotAuto2 extends OpMode {
    private Follower follower;
    private PathChain path1, path2;
    private final Pose startPose = new Pose(20.9, 121.5, Math.toRadians(144));
    private final Pose shootPose = new Pose(22.9, 119.5, Math.toRadians(148)); // Blue Scoring Pose
    private final Pose autoAStartPose = new Pose(48, 84, Math.toRadians(180)); // Blue position to start intaking balls in first section auto
    private Limelight3A limelight;

    public int state = 0;
    // Declare end-effector members
    private DcMotorEx intake = null;
    private DcMotorEx catapult1 = null;
    private DcMotorEx catapult2 = null;
    private static Servo foot = null;

    // motor power 1 = 100% and 0.5 = 50%
    // negative values = reverse ex: -0.5 = reverse 50%
    private double INTAKE_IN_POWER = -1;
    private double INTAKE_OUT_POWER = 0.9;
    private double INTAKE_OFF_POWER = 0.0;
    private double intakePower = INTAKE_OFF_POWER;

    private double CATAPULT_UP_POWER = -1;
    private double CATAPULT_DOWN_POWER = 1;
    private double CATAPULT_HOLD_POWER = 0.0;
    private double CATAPULT_HOLD_DOWN_POWER = 0.2;

    private double footPosition = 0.0;
    private double FOOT_UP_POSITION = 0.2;
    private double FOOT_DOWN_POSITION = 0.35;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, autoAStartPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), autoAStartPose.getHeading())
                .build();

        Drawing.init();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        intake = (DcMotorEx)hardwareMap.get(DcMotor.class, "intake");
        catapult1 = (DcMotorEx)hardwareMap.get(DcMotor.class, "rcat");
        catapult2 = (DcMotorEx)hardwareMap.get(DcMotor.class, "lcat");

        intake.setDirection(DcMotor.Direction.FORWARD); // Forward should INTAKE.
        catapult1.setDirection(DcMotor.Direction.REVERSE); // Backwards should pivot DOWN, or in the stowed position.
        catapult2.setDirection(DcMotor.Direction.FORWARD);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        catapult1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        catapult2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        foot = hardwareMap.get(Servo.class, "foot");
        footPosition = FOOT_UP_POSITION;
        foot.setPosition(footPosition);
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        updatePoseFromLL();
        Drawing.drawDebug(follower);

        telemetryM.update();
        switch (state) {
            case 0:
                follower.followPath(path1, true);
                state = 1;
                break;
            case 1:
                if (!follower.isBusy()) {
                    shootCatapult();
                    shootCatapult();
                    follower.followPath(path2, true);
                    state = 2;
                }
                break;
            case 2:
                break;
                /*
                if (!follower.isBusy()) {
                    intake.setPower(1);
                    follower.followPath(pathArray[AutoTarget.BLUE_AUTO_A_END.value].get());
                    state = 3;
                }
                break;
            case 3:
                if (!follower.isBusy()){
                    intake.setPower(0);
                    follower.followPath(pathArray[AutoTarget.BLUE_SCORING.value].get());
                    state = 4;
                }
                break;
            case 4:
                if (!follower.isBusy()){
                    shootCatapult();
                    state = 5;
                }
                break;
            case 5:
                break;
                */
            default:
                break;
        }

        telemetry.addData("Foot", footPosition);
        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetry.update();
    }

    // Returns Limelight pose in Pedro’s coordinate system if Limelight
    // sees an AprilTag, otherwise returns null
    private Pose getRobotPoseFromCamera() {
        LLResult result = limelight.getLatestResult();
        // If nothing useful, just fall back to follower’s current estimate
        if (result == null || !result.isValid()) {
            return null;
        }

        // Prefer MegaTag2 if you’re feeding IMU into Limelight; otherwise fall back to MT1
        /*Pose3D botpose = result.getBotpose_MT2();
        if (botpose == null) {
            botpose = result.getBotpose();
        }
        */
        Pose3D llpose = result.getBotpose();
        if (llpose == null) {
            return null;
        }

        double xMeters = llpose.getPosition().x;
        double yMeters = llpose.getPosition().y;

        // Convert to inches for Pedro & realign coordinate system to match pedro
        double xInches = 72 + DistanceUnit.METER.toInches(yMeters);
        double yInches = 72 - DistanceUnit.METER.toInches(xMeters);

        // --- 2) Heading (yaw) --- & convert to Pedro coordinate system
        YawPitchRollAngles ypr = llpose.getOrientation();
        double headingRad = ypr.getYaw(AngleUnit.RADIANS) -Math.toRadians(90);

        Pose pedroPose = new Pose(xInches, yInches, headingRad);

        return pedroPose;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private void updatePoseFromLL() {
        Pose llPose = getRobotPoseFromCamera();

        if (llPose != null) {
            telemetry.addLine("LL Data Valid 2");
            // Hard update position when left bumper is pressed
            if (gamepad1.leftBumperWasPressed()) {
                follower.setPose(llPose);
            } else { // Gentle correction when left bumper not pressed
                Pose odomPose = follower.getPose();       // Prediction (Pinpoint)

                // 1) Compute the difference between LL and odom
                double dx = llPose.getX() - odomPose.getX();
                double dy = llPose.getY() - odomPose.getY();
                double dH = AngleUnit.normalizeRadians(llPose.getHeading() - odomPose.getHeading());

                // 2) Gate out obviously bogus fixes
                double posError = Math.hypot(dx, dy);
                double headingError = Math.abs(dH);
                telemetry.addData("posError", posError);
                telemetry.addData("headingError", headingError);
                if (posError < 24 && headingError < Math.toRadians(45)) { // e.g. < 2 ft and < 45 degree err
                    double kPos = 0.00;   // how aggressively you correct position
                    double kH = 0.00;  // how aggressively you correct heading

                    Pose fused = new Pose(
                            odomPose.getX() + kPos * dx,
                            odomPose.getY() + kPos * dy,
                            odomPose.getHeading() + kH * dH
                    );

                    follower.setPose(fused);
                } else {
                    telemetry.addLine("pos or Heading Err too large.  Not updating");
                }
            }
        } else {
            telemetry.addLine("No LL Data");
        }
    }

    private void shootCatapult() {
        catapult1.setPower(CATAPULT_UP_POWER);
        catapult2.setPower(CATAPULT_UP_POWER);
        sleep(500);
        catapult1.setPower(CATAPULT_DOWN_POWER);
        catapult2.setPower(CATAPULT_DOWN_POWER);
        sleep(500);
        catapult1.setPower(CATAPULT_HOLD_DOWN_POWER);
        catapult2.setPower(CATAPULT_HOLD_DOWN_POWER);
    }

    static void sleep(int ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}