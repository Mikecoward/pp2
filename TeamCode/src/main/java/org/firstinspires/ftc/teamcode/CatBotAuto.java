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
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
@Autonomous(name = "CatBot Auto", group = "Autonomous")
public class CatBotAuto extends OpMode {
    private Follower follower;
    private boolean automatedDrive;
    private AutoTarget currentAutoTarget = AutoTarget.NONE;

    private int numPaths = 10;

    private PathChain pathChains[];
    public static Pose poseArray[] = {
            new Pose(20.2, 122.4, Math.toRadians(144)), // Blue Start Pose
            new Pose(25, 120, Math.toRadians(144)), // Blue Scoring Pose
            new Pose(40, 34, Math.toRadians(-131)), // Blue Parking Pose
            new Pose(46, 84, Math.toRadians(180)), // Blue position to start intaking balls in first section auto
            new Pose(22, 84, Math.toRadians(180)), // Blue position to stop intaking balls in first section auto
            new Pose(46, 60, Math.toRadians(180)), // Blue position to start intaking balls in second section auto
            new Pose(22, 60, Math.toRadians(180)), // Blue position to stop intaking balls in second section auto
            new Pose(46, 36, Math.toRadians(180)), // Blue position to start intaking balls in third section auto
            new Pose(22, 36, Math.toRadians(180)), // Blue position to stop intaking balls in third section auto
    };

    //private final Pose startPose = new Pose(20.9, 121.5, Math.toRadians(144));
    //private final Pose shootPose = new Pose(23.9, 120.5, Math.toRadians(148)); // Blue Scoring Pose
    //private final Pose autoAStartPose = new Pose(48, 84, Math.toRadians(180)); // Blue position to start intaking balls in first section auto


    private enum AutoTarget {
        NONE(-1),
        BLUE_STARTING(0),
        BLUE_SCORING(1),
        BLUE_PARKING(2),

        BLUE_AUTO_A_START (3),
        BLUE_AUTO_A_END (4),
        BLUE_AUTO_B_START (5),
        BLUE_AUTO_B_END (6),
        BLUE_AUTO_C_START (7),
        BLUE_AUTO_C_END (8);
        public final int idx;

        AutoTarget(int idx) {
            this.idx = idx;
        }
    }


    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    org.firstinspires.ftc.teamcode.pedroPathing.ActionScheduler scheduler = new ActionScheduler();
    private Limelight3A limelight;

    // Smooth command state
    private double cmdX = 0.0;
    private double cmdY = 0.0;
    private double cmdTurn = 0.0;

    // How fast commands are allowed to change per loop
    // Smaller = softer accel/decel, larger = snappier
    private static final double JOYSTICK_SLEW = 0.05;

    private Timer stateTimer =  new Timer();

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
    private double CATAPULT_HOLD_DOWN_POWER = 0.0;

    private double footPosition = 0.0;
    private double FOOT_UP_POSITION = 0.2;
    private double FOOT_DOWN_POSITION = 0.35;
    @Override


    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(poseArray[AutoTarget.BLUE_STARTING.idx]);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        buildPaths();

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
        catapultDown();
        scheduler.atSec(getRuntime() + 0.25,  this::catapultHold);
    }

    @Override
    //Call this once per loop
    public void loop() {
        follower.update();
        //updatePoseFromLL();
        Drawing.drawDebug(follower);
        telemetry.addData("state", state);
        telemetryM.update();
        scheduler.update(getRuntime());
        switch (state) {
            case 0:
                follower.followPath(pathChains[0], true);
                state = 1;
                break;
            case 1:
                if (!follower.isBusy()) {
                    shootCatapult();
                    scheduler.atSec(getRuntime() + 1.0,  this::setstate2);
                    state=100;
                }
                break;
            case 2:
                follower.followPath(pathChains[1], true);
                state = 3;
                break;
            case 3:
                if (!follower.isBusy()) {
                    intakeIn();
                    follower.followPath(pathChains[2], 0.3, true);
                    state = 4;
                }
                break;
            case 4:
                if (!follower.isBusy()){
                    scheduler.atSec(getRuntime() + 2,  this::intakeOff);
                    follower.followPath(pathChains[3],true);
                    state = 5;
                }
                break;
            case 5:
                if (!follower.isBusy()){
                    shootCatapult();
                    scheduler.atSec(getRuntime() + 1.0,  this::setstate6);
                    state = 100;
                }
                break;
            case 6:
                follower.followPath(pathChains[4], true);
                state = 7;
                break;
            case 7:
                if (!follower.isBusy()) {
                    intakeIn();
                    follower.followPath(pathChains[5], 0.3, true);
                    state = 8;
                }
                break;
            case 8:
                if (!follower.isBusy()){
                    scheduler.atSec(getRuntime() + 2,  this::intakeOff);
                    follower.followPath(pathChains[6],true);
                    state = 9;
                }
                break;
            case 9:
                if (!follower.isBusy()){
                    shootCatapult();
                    scheduler.atSec(getRuntime() + 1.0,  this::setstate10);
                    state = 100;

                }
                break;
            case 10:
                follower.followPath(pathChains[7], true);
                state = 11;
                break;
            case 11:
                if (!follower.isBusy()) {
                    intakeIn();
                    follower.followPath(pathChains[8], 0.3, true);
                    state = 12;
                }
                break;
            case 12:
                if (!follower.isBusy()){
                    scheduler.atSec(getRuntime() + 2,  this::intakeOff);
                    follower.followPath(pathChains[9],true);
                    state = 12;
                }
                break;
            default:
                break;
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

    public void setState(int state) {
        this.state = state;
        stateTimer.resetTimer();
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
        double now = getRuntime();

        scheduler.atSec(now,        this::catapultUp);
        scheduler.atSec(now + 0.5,  this::catapultDown);
        scheduler.atSec(now + 1.0,  this::catapultHold);
    }
    private void catapultUp() {
        catapult1.setPower(CATAPULT_UP_POWER);
        catapult2.setPower(CATAPULT_UP_POWER);
    }

    private void catapultDown() {
        catapult1.setPower(CATAPULT_DOWN_POWER);
        catapult2.setPower(CATAPULT_DOWN_POWER);
    }

    private void catapultHold() {
        catapult1.setPower(CATAPULT_HOLD_DOWN_POWER);
        catapult2.setPower(CATAPULT_HOLD_DOWN_POWER);
    }

    private void intakeIn() {
        intake.setPower(INTAKE_IN_POWER);
    }

    private void intakeOff() {
        intake.setPower(INTAKE_OFF_POWER);
    }

    private void setstate2() {
        state = 2;
    }
    private void setstate6() {
        state = 6;
    }
    private void setstate10() {
        state = 10;
    }


    static void sleep(int ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    private void buildPaths() {
        pathChains = new PathChain[numPaths];

        pathChains[0] = simplePathChain(AutoTarget.BLUE_STARTING.idx, AutoTarget.BLUE_SCORING.idx,0.8);

                /*follower.pathBuilder()
                        .addPath(new BezierLine(poseArray[AutoTarget.BLUE_STARTING.idx], poseArray[]))
                        .setLinearHeadingInterpolation(poseArray[AutoTarget.BLUE_STARTING.idx].getHeading(), poseArray[1].getHeading(),0.8)
                        .build();

                 */
        pathChains[1] = simplePathChain(AutoTarget.BLUE_SCORING.idx, AutoTarget.BLUE_AUTO_A_START.idx,0.4);

        /*pathChains[1] = follower.pathBuilder()
                .addPath(new BezierLine(poseArray[1], poseArray[3]))
                .setLinearHeadingInterpolation(poseArray[1].getHeading(), poseArray[3].getHeading(),0.4)
                .build();
         */

        pathChains[2] = simplePathChain(AutoTarget.BLUE_AUTO_A_START.idx, AutoTarget.BLUE_AUTO_A_END.idx,0.8);
/*
        pathChains[2] = follower.pathBuilder()
                .addPath(new BezierLine(poseArray[3], poseArray[4]))
                .setLinearHeadingInterpolation(poseArray[3].getHeading(), poseArray[4].getHeading(),0.8)
                .build();

 */
        pathChains[3] = simplePathChain(AutoTarget.BLUE_AUTO_A_END.idx, AutoTarget.BLUE_SCORING.idx,0.8);

        /*pathChains[3] = follower.pathBuilder()
                .addPath(new BezierLine(poseArray[3], poseArray[1]))
                .setLinearHeadingInterpolation(poseArray[3].getHeading(), poseArray[1].getHeading(),0.8)
                .build();

         */

        pathChains[4] = simplePathChain(AutoTarget.BLUE_SCORING.idx, AutoTarget.BLUE_AUTO_B_START.idx,0.4);
        pathChains[5] = simplePathChain(AutoTarget.BLUE_AUTO_B_START.idx, AutoTarget.BLUE_AUTO_B_END.idx,0.8);
        pathChains[6] = simplePathChain(AutoTarget.BLUE_AUTO_B_END.idx, AutoTarget.BLUE_SCORING.idx,0.8);
        /*
        pathChains[4] = follower.pathBuilder()
                .addPath(new BezierLine(poseArray[1], poseArray[4]))
                .setLinearHeadingInterpolation(poseArray[1].getHeading(), poseArray[4].getHeading(),0.8)
                .build();


        pathChains[5] = follower.pathBuilder()
                .addPath(new BezierLine(poseArray[4], poseArray[5]))
                .setLinearHeadingInterpolation(poseArray[4].getHeading(), poseArray[5].getHeading(),0.8)
                .build();
        pathChains[6] = follower.pathBuilder()
                .addPath(new BezierLine(poseArray[5], poseArray[1]))
                .setLinearHeadingInterpolation(poseArray[5].getHeading(), poseArray[1].getHeading(),0.8)
                .build();
         */
        pathChains[7] = simplePathChain(AutoTarget.BLUE_SCORING.idx, AutoTarget.BLUE_AUTO_C_START.idx,0.4);
        pathChains[8] = simplePathChain(AutoTarget.BLUE_AUTO_C_START.idx, AutoTarget.BLUE_AUTO_C_END.idx,0.8);
        pathChains[9] = simplePathChain(AutoTarget.BLUE_AUTO_C_END.idx, AutoTarget.BLUE_SCORING.idx,0.8);

        /*pathChains[7] = follower.pathBuilder()
                .addPath(new BezierLine(poseArray[1], poseArray[6]))
                .setLinearHeadingInterpolation(poseArray[1].getHeading(), poseArray[6].getHeading(),0.8)
                .build();
        pathChains[8] = follower.pathBuilder()
                .addPath(new BezierLine(poseArray[6], poseArray[7]))
                .setLinearHeadingInterpolation(poseArray[6].getHeading(), poseArray[7].getHeading(),0.8)
                .build();
*/
    }

    private PathChain simplePathChain(int start, int end, double headingTime) {
        return follower.pathBuilder()
                .addPath(new BezierLine(poseArray[start], poseArray[end]))
                .setLinearHeadingInterpolation(poseArray[start].getHeading(), poseArray[end].getHeading(),headingTime)
                .build();
    }
}