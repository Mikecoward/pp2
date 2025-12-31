package org.firstinspires.ftc.teamcode;

//import org.firstinspires.ftc.teamcode.pedroPathing.Tuning.telemetryM;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;
import org.firstinspires.ftc.teamcode.PoseStorage;


@Configurable
public abstract class basecatbotAuto2 extends OpMode {

    public enum Alliance { BLUE, RED }

    // ---- override in subclasses ----
    protected abstract Alliance getAlliance();

    // ---- field mirror config ----
    private static final double FIELD_SIZE_IN = 144.0; // 12 ft
    // Blue->Red mirror: y' = 144 - y, heading' = PI-heading
    protected Pose mirrorBlueToRed(Pose bluePose) {
        double x = FIELD_SIZE_IN - bluePose.getX();
        double y = bluePose.getY();
        double h = AngleUnit.normalizeRadians(Math.PI-bluePose.getHeading());
        return new Pose(x, y, h);
    }

    // ---- your members ----
    protected Follower follower;
    protected boolean automatedDrive;
    protected AutoTarget currentAutoTarget = AutoTarget.NONE;

    // FIX: you index up to 18, so this must be >= 19
    protected int numPaths = 2;
    protected double speedfactor = 0.20;
    protected PathChain[] pathChains;

    // ---- IMPORTANT ----
    // These are your BLUE poses (source of truth). RED is derived by mirroring.
    // Your snippet is missing poses for indices 10 and 11. Fill them in.
    protected static final Pose[] poseArrayBlue = {
            new Pose(96, 9, Math.toRadians(90)),   // 0 Blue Start Pose
            new Pose(105, 9 , Math.toRadians(90)),  // 1 Dummy Position
    };

    // Runtime pose array for the selected alliance
    protected Pose[] poseArray;

    protected enum AutoTarget {
        NONE(-1),
        STARTING(0),
        DUMMY(1);
        public final int idx;
        AutoTarget(int idx) { this.idx = idx; }
    }

    protected boolean slowMode = false;
    protected double slowModeMultiplier = 0.5;
    protected ActionScheduler scheduler = new ActionScheduler();
    protected Limelight3A limelight;

    protected Timer stateTimer = new Timer();
    public int state = 0;

    // End-effector
    protected DcMotorEx intake = null;
    protected DcMotorEx catapult1 = null;
    protected DcMotorEx catapult2 = null;
    protected Servo foot = null;

    protected double INTAKE_IN_POWER = -1;
    protected double INTAKE_OUT_POWER = 0.9;
    protected double INTAKE_OFF_POWER = 0.0;

    protected double CATAPULT_UP_POWER = -1;
    protected double CATAPULT_DOWN_POWER = 1;
    protected double CATAPULT_HOLD_DOWN_POWER = 0.0;

    protected double footPosition = 0.0;
    protected double FOOT_UP_POSITION = 0.2;
    protected double FOOT_DOWN_POSITION = 0.35;

    @Override
    public void init() {
        // Build alliance-specific pose array (BLUE = as-is, RED = mirrored-from-blue)
        Alliance alliance = getAlliance();
        poseArray = new Pose[poseArrayBlue.length];
        for (int i = 0; i < poseArrayBlue.length; i++) {
            poseArray[i] = (alliance == Alliance.BLUE) ? poseArrayBlue[i] : mirrorBlueToRed(poseArrayBlue[i]);
        }

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(poseArray[AutoTarget.STARTING.idx]);
        follower.update();

        //telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        buildPaths();
        //Drawing.init();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        intake = (DcMotorEx) hardwareMap.get(DcMotor.class, "intake");
        catapult1 = (DcMotorEx) hardwareMap.get(DcMotor.class, "rcat");
        catapult2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "lcat");

        intake.setDirection(DcMotor.Direction.FORWARD);
        catapult1.setDirection(DcMotor.Direction.REVERSE);
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
        scheduler.atSec(getRuntime() + 0.25, this::catapultHold);
    }

    @Override
    public void loop() {
        follower.update();
        //Drawing.drawDebug(follower);

        telemetry.addData("Alliance", getAlliance());
        telemetry.addData("state", state);
        PoseStorage.currentPose = follower.getPose();

        //telemetryM.update();
        scheduler.update(getRuntime());

        switch (state) {
            case 0:
                follower.followPath(pathChains[0], true);
                state = 100;
                break;
            case 100:  // Sit here and wait
                break;
            default:
                break;
        }

        telemetry.addData("Foot", footPosition);
        //telemetryM.debug("position", follower.getPose());
        //telemetryM.debug("velocity", follower.getVelocity());
        //telemetryM.debug("automatedDrive", automatedDrive);
        //telemetryM.debug("autoTarget", currentAutoTarget);
        telemetry.update();
    }

    // ---- Limelight pose (unchanged) ----
    protected Pose getRobotPoseFromCamera() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;

        Pose3D llpose = result.getBotpose();
        if (llpose == null) return null;

        double xMeters = llpose.getPosition().x;
        double yMeters = llpose.getPosition().y;

        double xInches = 72 + DistanceUnit.METER.toInches(yMeters);
        double yInches = 72 - DistanceUnit.METER.toInches(xMeters);

        YawPitchRollAngles ypr = llpose.getOrientation();
        double headingRad = ypr.getYaw(AngleUnit.RADIANS) - Math.toRadians(90);

        return new Pose(xInches, yInches, headingRad);
    }

    // ---- actions ----
    protected void shootCatapult() {
        double now = getRuntime();
        scheduler.atSec(now, this::catapultUp);
        scheduler.atSec(now + 0.5, this::catapultDown);
        scheduler.atSec(now + 1.0, this::catapultHold);
    }
    protected void shootCatapultnew() {
        double now = getRuntime();
        scheduler.atSec(now, this::catapultUp);
        scheduler.atSec(now + 0.5, this::catapultHold);
        scheduler.atSec(now + 1.5, this::catapultDown);
        scheduler.atSec(now + 2, this::catapultHold);
    }

    protected void catapultUp() {
        catapult1.setPower(CATAPULT_UP_POWER);
        catapult2.setPower(CATAPULT_UP_POWER);
    }

    protected void catapultDown() {
        catapult1.setPower(CATAPULT_DOWN_POWER);
        catapult2.setPower(CATAPULT_DOWN_POWER);
    }

    protected void catapultHold() {
        catapult1.setPower(CATAPULT_HOLD_DOWN_POWER);
        catapult2.setPower(CATAPULT_HOLD_DOWN_POWER);
    }

    protected void intakeIn() { intake.setPower(INTAKE_IN_POWER); }
    protected void intakeOff() { intake.setPower(INTAKE_OFF_POWER); }

    protected void setstate2() { state = 2; }
    protected void setstate6() { state = 6; }
    protected void setstate10() { state = 10; }

    // ---- paths ----
    protected void buildPaths() {
        pathChains = new PathChain[numPaths];

        pathChains[0]  = simplePathChain(AutoTarget.STARTING.idx,     AutoTarget.DUMMY.idx,     0.8);
    }

    protected PathChain simplePathChain(int start, int end, double headingTime) {
        return follower.pathBuilder()
                .addPath(new BezierLine(poseArray[start], poseArray[end]))
                .setLinearHeadingInterpolation(
                        poseArray[start].getHeading(),
                        poseArray[end].getHeading(),
                        headingTime
                )
                .build();
    }
}