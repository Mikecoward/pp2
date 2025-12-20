package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.telemetryM;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.function.Supplier;

@Configurable
public abstract class BaseCatBotTeleop extends OpMode {

    public enum Alliance { BLUE, RED }
    protected abstract Alliance getAlliance();

    // Mirror across Y axis centerline (x=72): x' = 144 - x, y' = y, heading' = pi - heading
    private static final double FIELD_SIZE_IN = 144.0;

    protected Pose mirrorBlueToRed(Pose bluePose) {
        double x = FIELD_SIZE_IN - bluePose.getX();
        double y = bluePose.getY();
        double h = AngleUnit.normalizeRadians(Math.PI - bluePose.getHeading());
        return new Pose(x, y, h);
    }

    // --- Original members ---
    protected Follower follower;
    protected boolean automatedDrive;
    protected AutoTarget currentAutoTarget = AutoTarget.NONE;

    protected int numPaths = 5;
    protected Supplier<PathChain>[] pathArray;

    // BLUE “source of truth”
    protected static final Pose[] poseArrayBlue = {
            new Pose(25.1, 129.3, Math.toRadians(144)), // 0 Blue Start Pose
            new Pose(24.7, 121.5, Math.toRadians(144)), // 1 Blue Scoring Pose
            new Pose(40,   34,    Math.toRadians(-131)),// 2 Blue Parking Pose
            new Pose(48,   90,    Math.toRadians(-180)),// 3 Blue Auto A Start
            new Pose(30,   90,    Math.toRadians(-180)) // 4 Blue Auto A End
    };

    // Alliance-specific poses (computed at init)
    protected Pose[] poseArray;

    protected enum AutoTarget {
        NONE(-1),
        STARTING(0),
        SCORING(1),
        PARKING(2),
        AUTO_A_START(3),
        AUTO_A_END(4);

        public final int value;
        AutoTarget(int value) { this.value = value; }
    }

    protected boolean slowMode = false;
    protected double slowModeMultiplier = 0.5;

    protected Limelight3A limelight;

    // Smooth command state
    protected double cmdX = 0.0;
    protected double cmdY = 0.0;
    protected double cmdTurn = 0.0;
    protected static final double JOYSTICK_SLEW = 0.05;

    // End-effector
    protected DcMotorEx intake = null;
    protected DcMotorEx catapult1 = null;
    protected DcMotorEx catapult2 = null;
    protected Servo foot = null;

    protected double INTAKE_IN_POWER = -1;
    protected double INTAKE_OFF_POWER = 0.0;

    protected double CATAPULT_UP_POWER = -1;
    protected double CATAPULT_DOWN_POWER = 1;
    protected double CATAPULT_HOLD_DOWN_POWER = 0.0;

    protected double footPosition = 0.0;
    protected double FOOT_UP_POSITION = 0.2;
    protected double FOOT_DOWN_POSITION = 0.35;

    @Override
    public void init() {
        // Build alliance-specific pose array
        poseArray = new Pose[poseArrayBlue.length];
        for (int i = 0; i < poseArrayBlue.length; i++) {
            poseArray[i] = (getAlliance() == Alliance.BLUE) ? poseArrayBlue[i] : mirrorBlueToRed(poseArrayBlue[i]);
        }

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(poseArray[AutoTarget.STARTING.value]);
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathArray = new Supplier[numPaths];
        for (int i = 0; i < numPaths; i++) {
            final int index = i;
            pathArray[index] = () -> follower.pathBuilder()
                    .addPath(new Path(new BezierLine(follower::getPose, poseArray[index])))
                    .setHeadingInterpolation(
                            HeadingInterpolator.linearFromPoint(
                                    follower::getHeading,
                                    poseArray[index].getHeading(),
                                    0.8
                            )
                    )
                    .build();
        }

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
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        updatePoseFromLL();
        Drawing.drawDebug(follower);

        telemetry.addData("Alliance", getAlliance());
        telemetryM.update();

        if (!automatedDrive) {
            double targetY    = gamepad1.right_stick_y * Math.pow(Math.abs(gamepad1.right_stick_y), 1.2);
            double targetX    = gamepad1.right_stick_x * Math.pow(Math.abs(gamepad1.right_stick_x), 1.2);
            double targetTurn = gamepad1.left_stick_x  * Math.pow(Math.abs(gamepad1.left_stick_x),  1.5);

            cmdY    += clamp(targetY    - cmdY,    -JOYSTICK_SLEW, JOYSTICK_SLEW);
            cmdX    += clamp(targetX    - cmdX,    -JOYSTICK_SLEW, JOYSTICK_SLEW);
            cmdTurn += clamp(targetTurn - cmdTurn, -JOYSTICK_SLEW, JOYSTICK_SLEW);

            double mult = slowMode ? slowModeMultiplier : 1.0;
            follower.setTeleOpDrive(
                    -cmdY * mult,
                    -cmdX * mult,
                    -cmdTurn * mult,
                    true // Robot centric (as you had)
            );
        }

        // A -> scoring
        if (gamepad1.aWasPressed() && !automatedDrive) {
            follower.followPath(pathArray[AutoTarget.SCORING.value].get());
            automatedDrive = true;
            currentAutoTarget = AutoTarget.SCORING;
        }
        if (!gamepad1.a && automatedDrive && currentAutoTarget == AutoTarget.SCORING) {
            follower.startTeleopDrive();
            automatedDrive = false;
            currentAutoTarget = AutoTarget.NONE;
        }

        // B -> parking
        if (gamepad1.bWasPressed() && !automatedDrive) {
            follower.followPath(pathArray[AutoTarget.PARKING.value].get());
            automatedDrive = true;
            currentAutoTarget = AutoTarget.PARKING;
        }
        if (!gamepad1.b && automatedDrive && currentAutoTarget == AutoTarget.PARKING) {
            follower.startTeleopDrive();
            automatedDrive = false;
            currentAutoTarget = AutoTarget.NONE;
        }

        // X -> auto A start
        if (gamepad1.xWasPressed() && !automatedDrive) {
            follower.followPath(pathArray[AutoTarget.AUTO_A_START.value].get());
            automatedDrive = true;
            currentAutoTarget = AutoTarget.AUTO_A_START;
        }
        if (!gamepad1.x && automatedDrive && currentAutoTarget == AutoTarget.AUTO_A_START) {
            follower.startTeleopDrive();
            automatedDrive = false;
            currentAutoTarget = AutoTarget.NONE;
        }

        // Y -> auto A end
        if (gamepad1.yWasPressed() && !automatedDrive) {
            follower.followPath(pathArray[AutoTarget.AUTO_A_END.value].get());
            automatedDrive = true;
            currentAutoTarget = AutoTarget.AUTO_A_END;
        }
        if (!gamepad1.y && automatedDrive && currentAutoTarget == AutoTarget.AUTO_A_END) {
            follower.startTeleopDrive();
            automatedDrive = false;
            currentAutoTarget = AutoTarget.NONE;
        }

        if (gamepad1.rightBumperWasPressed()) {
            shootCatapult();
        }

        if (gamepad1.left_bumper) {
            intake.setPower(INTAKE_IN_POWER);
            telemetry.addData("Intake Vel/mA", "%4.2f, %4.2f",
                    intake.getVelocity(), intake.getCurrent(CurrentUnit.MILLIAMPS));
        } else if (gamepad1.left_trigger > 0.1) {
            intake.setPower(gamepad1.left_trigger);
            telemetry.addData("Intake Vel/mA", "%4.2f, %4.2f",
                    intake.getVelocity(), intake.getCurrent(CurrentUnit.MILLIAMPS));
        } else {
            intake.setPower(INTAKE_OFF_POWER);
        }

        if (gamepad1.dpad_down) {
            footPosition = FOOT_DOWN_POSITION;
            foot.setPosition(footPosition);
        }
        if (gamepad1.dpad_up) {
            footPosition = FOOT_UP_POSITION;
            foot.setPosition(footPosition);
        }

        Pose odomPose = follower.getPose();
        telemetry.addData("PP X/Y/H", "%4.2f, %4.2f, %4.1f°",
                odomPose.getX(), odomPose.getY(), Math.toDegrees(odomPose.getHeading()));

        telemetry.addData("Foot", footPosition);
        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
        telemetryM.debug("autoTarget", currentAutoTarget);
        telemetry.update();
    }

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
        double headingRad = AngleUnit.normalizeRadians(ypr.getYaw(AngleUnit.RADIANS) - Math.toRadians(90));

        return new Pose(xInches, yInches, headingRad);
    }

    protected void updatePoseFromLL() {
        Pose llPose = getRobotPoseFromCamera();
        if (llPose != null) {
            telemetry.addLine("LL Data Valid");
            if (gamepad1.leftBumperWasPressed()) {
                follower.setPose(llPose);
            } else {
                telemetry.addData("LL X/Y/H", "%4.2f, %4.2f, %4.1f°",
                        llPose.getX(), llPose.getY(), Math.toDegrees(llPose.getHeading()));
            }
        } else {
            telemetry.addLine("No LL Data");
        }
    }

    protected void shootCatapult() {
        catapult1.setPower(CATAPULT_UP_POWER);
        catapult2.setPower(CATAPULT_UP_POWER);
        sleep(500);
        catapult1.setPower(CATAPULT_DOWN_POWER);
        catapult2.setPower(CATAPULT_DOWN_POWER);
        sleep(500);
        catapult1.setPower(CATAPULT_HOLD_DOWN_POWER);
        catapult2.setPower(CATAPULT_HOLD_DOWN_POWER);
    }

    protected static void sleep(int ms) {
        try { Thread.sleep(ms); }
        catch (InterruptedException e) { Thread.currentThread().interrupt(); }
    }

    protected double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}