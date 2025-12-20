package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.telemetryM;

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

@Configurable
public abstract class BaseCatBotAuto extends OpMode {

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
    protected int numPaths = 19;
    protected double speedfactor = 0.30;
    protected PathChain[] pathChains;

    // ---- IMPORTANT ----
    // These are your BLUE poses (source of truth). RED is derived by mirroring.
    // Your snippet is missing poses for indices 10 and 11. Fill them in.
    protected static final Pose[] poseArrayBlue = {
            new Pose(25.1, 129.3, Math.toRadians(144)),   // 0 Blue Start Pose
            new Pose(24.7, 121.5 , Math.toRadians(144)),  // 1 Blue Scoring Pose
            new Pose(40,   34,    Math.toRadians(-131)),  // 2 Blue Parking Pose
            new Pose(43,   88,    Math.toRadians(180)),   // 3 Blue intake A start
            new Pose(20,   88,    Math.toRadians(180)),   // 4 Blue intake A end
            new Pose(46,   65,    Math.toRadians(180)),   // 5 Blue intake B start
            new Pose(20,   65,    Math.toRadians(180)),   // 6 Blue intake B end
            new Pose(46,   40,    Math.toRadians(180)),   // 7 Blue intake C start
            new Pose(20,   40,    Math.toRadians(180)),   // 8 Blue intake C end
            new Pose(43,   105,   Math.toRadians(180)),   // 9 Line segment intermediate

            // TODO: YOU MUST ADD THESE (your enum references 10 and 11)
            // 10 Gate Start
            new Pose(0, 0, 0),
            // 11 Gate End
            new Pose(0, 0, 0),
    };

    // Runtime pose array for the selected alliance
    protected Pose[] poseArray;

    protected enum AutoTarget {
        NONE(-1),
        STARTING(0),
        SCORING(1),
        PARKING(2),

        AUTO_A_START(3),
        AUTO_A_END(4),
        AUTO_B_START(5),
        AUTO_B_END(6),
        AUTO_C_START(7),
        AUTO_C_END(8),
        AUTO_LINE_SEG(9),

        AUTO_GATE_START(10),
        AUTO_GATE_END(11);

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

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        buildPaths();
        Drawing.init();

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
        Drawing.drawDebug(follower);

        telemetry.addData("Alliance", getAlliance());
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
                    scheduler.atSec(getRuntime() + 1.0, this::setstate2);
                    state = 100;
                }
                break;

            case 2:
                follower.followPath(pathChains[10], true);
                state = 31;
                break;

            case 31:
                if (!follower.isBusy()) {
                    follower.followPath(pathChains[11], true);
                    state = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    intakeIn();
                    follower.followPath(pathChains[2], speedfactor, true);
                    state = 4;
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    scheduler.atSec(getRuntime() + 2, this::intakeOff);
                    follower.followPath(pathChains[16], true);
                    state = 5;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    shootCatapult();
                    scheduler.atSec(getRuntime() + 1.0, this::setstate6);
                    state = 100;
                }
                break;

            case 6:
                follower.followPath(pathChains[10], true);
                state = 71;
                break;

            case 71:
                if (!follower.isBusy()) {
                    follower.followPath(pathChains[12], true);
                    state = 7;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    intakeIn();
                    follower.followPath(pathChains[5], speedfactor, true);
                    state = 8;
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    scheduler.atSec(getRuntime() + 2, this::intakeOff);
                    follower.followPath(pathChains[14], true);
                    state = 81;
                }
                break;

            case 81:
                if (!follower.isBusy()) {
                    follower.followPath(pathChains[6], true);
                    state = 9;
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    shootCatapult();
                    scheduler.atSec(getRuntime() + 1.0, this::setstate10);
                    state = 100;
                }
                break;

            case 10:
                follower.followPath(pathChains[10], true);
                state = 101;
                break;

            case 101:
                if (!follower.isBusy()) {
                    follower.followPath(pathChains[13], true);
                    state = 11;
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    intakeIn();
                    follower.followPath(pathChains[8], speedfactor, true);
                    state = 12;
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    scheduler.atSec(getRuntime() + 2, this::intakeOff);
                    follower.followPath(pathChains[15], true);
                    state = 13;
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    follower.followPath(pathChains[9], true);
                    state = 14;
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

        pathChains[0]  = simplePathChain(AutoTarget.STARTING.idx,     AutoTarget.SCORING.idx,     0.8);
        pathChains[1]  = simplePathChain(AutoTarget.SCORING.idx,      AutoTarget.AUTO_A_START.idx,0.4);
        pathChains[2]  = simplePathChain(AutoTarget.AUTO_A_START.idx, AutoTarget.AUTO_A_END.idx,  0.8);
        pathChains[3]  = simplePathChain(AutoTarget.AUTO_A_END.idx,   AutoTarget.SCORING.idx,     0.8);

        pathChains[4]  = simplePathChain(AutoTarget.SCORING.idx,      AutoTarget.AUTO_B_START.idx,0.4);
        pathChains[5]  = simplePathChain(AutoTarget.AUTO_B_START.idx, AutoTarget.AUTO_B_END.idx,  0.8);
        pathChains[6]  = simplePathChain(AutoTarget.AUTO_B_START.idx, AutoTarget.SCORING.idx,     0.8);

        pathChains[7]  = simplePathChain(AutoTarget.SCORING.idx,      AutoTarget.AUTO_C_START.idx,0.4);
        pathChains[8]  = simplePathChain(AutoTarget.AUTO_C_START.idx, AutoTarget.AUTO_C_END.idx,  0.8);
        pathChains[9]  = simplePathChain(AutoTarget.AUTO_C_START.idx, AutoTarget.SCORING.idx,     0.8);

        pathChains[10] = simplePathChain(AutoTarget.SCORING.idx,      AutoTarget.AUTO_LINE_SEG.idx,0.8);
        pathChains[11] = simplePathChain(AutoTarget.AUTO_LINE_SEG.idx,AutoTarget.AUTO_A_START.idx, 0.8);
        pathChains[12] = simplePathChain(AutoTarget.AUTO_LINE_SEG.idx,AutoTarget.AUTO_B_START.idx, 0.8);
        pathChains[13] = simplePathChain(AutoTarget.AUTO_LINE_SEG.idx,AutoTarget.AUTO_C_START.idx, 0.8);

        pathChains[14] = simplePathChain(AutoTarget.AUTO_B_END.idx,   AutoTarget.AUTO_B_START.idx, 0.8);
        pathChains[15] = simplePathChain(AutoTarget.AUTO_C_END.idx,   AutoTarget.AUTO_C_START.idx, 0.8);
        pathChains[16] = simplePathChain(AutoTarget.AUTO_A_END.idx,   AutoTarget.SCORING.idx,      0.8);

        pathChains[17] = simplePathChain(AutoTarget.AUTO_GATE_START.idx, AutoTarget.AUTO_GATE_END.idx, 0.8);
        pathChains[18] = simplePathChain(AutoTarget.AUTO_GATE_END.idx,   AutoTarget.SCORING.idx,       0.8);
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