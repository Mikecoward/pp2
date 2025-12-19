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
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Configurable
@TeleOp(name = "CatBot", group = "Teleop")
public class PedroPathingStarter extends OpMode {
    private Follower follower;
    private boolean automatedDrive;
    private AutoTarget currentAutoTarget = AutoTarget.NONE;

    private int numPaths = 5;
    private Supplier<PathChain>[] pathArray;
    public static Pose poseArray[] = {
            new Pose(23.2, 125.6, Math.toRadians(148)), // Blue Start Pose
            new Pose(25.2, 123.6, Math.toRadians(148)), // Blue Scoring Pose
        new Pose(40, 34, Math.toRadians(-131)), // Blue Parking Pose
        new Pose(48, 90, Math.toRadians(-180)), // Blue position to start intaking balls in first section auto
        new Pose(30, 90, Math.toRadians(-180)), // Blue position to stop intaking balls in first section auto
    };

    private enum AutoTarget {
        NONE(-1),
        BLUE_STARTING(0),
        BLUE_SCORING(1),
        BLUE_PARKING(2),

        BLUE_AUTO_A_START (3),
        BLUE_AUTO_A_END (4);
        public final int value;

        AutoTarget(int value) {
            this.value = value;
        }
    }


    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    private Limelight3A limelight;

    // Smooth command state
    private double cmdX = 0.0;
    private double cmdY = 0.0;
    private double cmdTurn = 0.0;

    // How fast commands are allowed to change per loop
    // Smaller = softer accel/decel, larger = snappier
    private static final double JOYSTICK_SLEW = 0.05;


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
        follower.setStartingPose(poseArray[AutoTarget.BLUE_STARTING.value]);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        pathArray = new Supplier[5];
        for(int i = 0; i < numPaths; i++) {
            final int index = i; // Create a final or effectively final copy of 'i'
            pathArray[index] = () -> follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new Path(new BezierLine(follower::getPose, poseArray[index])))
                    .setHeadingInterpolation(
                            HeadingInterpolator.linearFromPoint(
                                    follower::getHeading,
                                    poseArray[index].getHeading(),     // <-- Use final pose's heading
                                    0.8                                // <-- reach near end of the path
                            )
                    ).build();
        }

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
    public void start() {
        follower.startTeleopDrive();
    }
    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        //updatePoseFromLL();
        Drawing.drawDebug(follower);

        telemetryM.update();
        if (!automatedDrive) {
            // Desired inputs from driver
            double targetY    = gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y);
            double targetX    = gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x);
            double targetTurn = gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x);

            telemetryM.debug("targetY", targetY);


            // Slew-limit the change in command each loop
            cmdY    += clamp(targetY    - cmdY,    -JOYSTICK_SLEW, JOYSTICK_SLEW);
            cmdX    += clamp(targetX    - cmdX,    -JOYSTICK_SLEW, JOYSTICK_SLEW);
            cmdTurn += clamp(targetTurn - cmdTurn, -JOYSTICK_SLEW, JOYSTICK_SLEW);

            if (!slowMode) {
                follower.setTeleOpDrive(
                        -cmdY,
                        -cmdX,
                        -cmdTurn,
                        true // Robot Centric
                );
            } else {
                follower.setTeleOpDrive(
                        -cmdY * slowModeMultiplier,
                        -cmdX * slowModeMultiplier,
                        -cmdTurn * slowModeMultiplier,
                        true // Robot Centric
                );
            }
        }

        // A: start moving toward BlueScoringPose when A is pressed
        if (gamepad1.aWasPressed() && !automatedDrive) {
            follower.followPath(pathArray[AutoTarget.BLUE_SCORING.value].get());
            automatedDrive = true;
            currentAutoTarget = AutoTarget.BLUE_SCORING;
        }

        // If A is released while we're running the BLUE_SCORING auto, stop immediately
        if (!gamepad1.a && automatedDrive && currentAutoTarget == AutoTarget.BLUE_SCORING) {
            follower.startTeleopDrive();
            automatedDrive = false;
            currentAutoTarget = AutoTarget.NONE;
        }

        // B: start moving toward startingPose when B is pressed (edge trigger),
        // and only if we're not already in an automated mode.
        if (gamepad1.bWasPressed() && !automatedDrive) {
            follower.followPath(pathArray[AutoTarget.BLUE_PARKING.value].get());
            automatedDrive = true;
            currentAutoTarget = AutoTarget.BLUE_PARKING;
        }

        // If B is released while we're running the STARTING auto, stop immediately
        if (!gamepad1.b && automatedDrive && currentAutoTarget == AutoTarget.BLUE_PARKING) {
            follower.startTeleopDrive();
            automatedDrive = false;
            currentAutoTarget = AutoTarget.NONE;
        }

        if (gamepad1.xWasPressed() && !automatedDrive) {
            follower.followPath(pathArray[AutoTarget.BLUE_AUTO_A_START.value].get());
            automatedDrive = true;
            currentAutoTarget = AutoTarget.BLUE_AUTO_A_START;
        }

        // If B is released while we're running the STARTING auto, stop immediately
        if (!gamepad1.x && automatedDrive && currentAutoTarget == AutoTarget.BLUE_AUTO_A_START) {
            follower.startTeleopDrive();
            automatedDrive = false;
            currentAutoTarget = AutoTarget.NONE;
        }

        if (gamepad1.yWasPressed() && !automatedDrive) {
            follower.followPath(pathArray[AutoTarget.BLUE_AUTO_A_END.value].get());
            automatedDrive = true;
            currentAutoTarget = AutoTarget.BLUE_AUTO_A_END;
        }

        // If B is released while we're running the STARTING auto, stop immediately
        if (!gamepad1.y && automatedDrive && currentAutoTarget == AutoTarget.BLUE_AUTO_A_END) {
            follower.startTeleopDrive();
            automatedDrive = false;
            currentAutoTarget = AutoTarget.NONE;
        }

        if (gamepad1.rightBumperWasPressed()) {
            shootCatapult();
        }

        if (gamepad1.left_bumper) {
            intake.setPower(INTAKE_IN_POWER);
            telemetry.addData("Intake Speed /mA", "%4.2f, %4.2f", intake.getVelocity(), intake.getCurrent(CurrentUnit.MILLIAMPS));
        } else if (gamepad1.left_trigger > 0.5) {
            intake.setPower(INTAKE_OUT_POWER);
            telemetry.addData("Intake Speed /mA", "%4.2f, %4.2f", intake.getVelocity(), intake.getCurrent(CurrentUnit.MILLIAMPS));
        } else {
            intake.setPower(INTAKE_OFF_POWER);
        }

        if (gamepad1.dpad_down) {
            footPosition = FOOT_DOWN_POSITION;
            foot.setPosition(footPosition);        }
        if (gamepad1.dpad_up) {
            footPosition = FOOT_UP_POSITION;
            foot.setPosition(footPosition);
        }
        telemetry.addData("Foot", footPosition);
        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
        telemetryM.debug("autoTarget", currentAutoTarget);
        telemetry.update();
    }

    // Returns Limelight pose in Pedro’s coordinate system if Limelight
    // sees an AprilTag, otherwise returns null
    private Pose getRobotPoseFromCamera() {
        LLResult result = limelight.getLatestResult();
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
        double headingRad = normalizeRadians(ypr.getYaw(AngleUnit.RADIANS)-Math.toRadians(90));

        return new Pose(xInches, yInches, headingRad);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private void updatePoseFromLL() {
        Pose llPose = getRobotPoseFromCamera();

        Pose odomPose = follower.getPose();       // Prediction (Pinpoint)
        telemetry.addData("PP X / Y", "%4.2f, %4.2f, %4.2f", odomPose.getX(), odomPose.getY(), odomPose.getHeading()*180/3.14);

        if (llPose != null) {
            telemetry.addLine("LL Data Valid");

            // Hard update position when left bumper is pressed
            if (gamepad1.leftBumperWasPressed()) {
                follower.setPose(llPose);
                //follower.resetHeading();
            } else { // Gentle correction when left bumper not pressed
                telemetry.addData("LL X / Y", "%4.2f, %4.2f, %4.2f", llPose.getX(), llPose.getY(), llPose.getHeading()*180/3.14);

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

    private static double normalizeDegrees(double degrees) {
        degrees = degrees % 360.0;
        if (degrees <= -180.0) {
            degrees += 360.0;
        } else if (degrees > 180.0) {
            degrees -= 360.0;
        }
        return degrees;
    }

    private static double normalizeRadians(double radians) {
        radians = radians % (2.0 * Math.PI);
        if (radians <= -Math.PI) {
            radians += 2.0 * Math.PI;
        } else if (radians > Math.PI) {
            radians -= 2.0 * Math.PI;
        }
        return radians;
    }
}