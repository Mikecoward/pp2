package org.firstinspires.ftc.teamcode;

//import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.telemetryM;

import com.bylazar.configurables.annotations.Configurable;
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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


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
    int upperlimit = 0;
    int lowerlimit = -1280;
    protected boolean automatedDrive;
    protected AutoTarget currentAutoTarget = AutoTarget.NONE;
    boolean robotCentric = false;
    protected int numPaths = 5;
    protected Supplier<PathChain>[] pathArray;

    // BLUE “source of truth”
    protected static final Pose[] poseArrayBlue = {
            new Pose(25.1, 129.3, Math.toRadians(144)), // 0 Blue Start Pose
            new Pose(29, 121.5, Math.toRadians(144)), // 1 Blue Scoring Pose
            new Pose(104 ,   34,    Math.toRadians(-131)),// 2 Blue Parking Pose
            new Pose(144-16,   16,    Math.toRadians(-180)),// 3 Blue Pickup Pose
            new Pose(30,   90,    Math.toRadians(-180)) //
    };
    protected int lifterTargetPosition = 0;
    protected static final int LIFTER_INCREMENT_TICKS = 10;
    // Alliance-specific poses (computed at init)
    protected Pose[] poseArray;

    protected enum AutoTarget {
        NONE(-1),
        STARTING(0),
        SCORING(1),
        PARKING(2),
        PICKUP(3),
        AUTO_A_END(4);

        public final int value;
        AutoTarget(int value) { this.value = value; }
    }

    protected boolean slowMode = false;
    protected double slowModeMultiplier = 0.5;

    // Speed cap for all drive movements (0.0 to 1.0)
    protected double driveSpeedCap = 0.5;
    // Demo mode - disables A/B/X/Y automated drive buttons
    protected boolean demoMode = false;

    protected Limelight3A limelight;

    // Smooth command state
    protected double cmdX = 0.0;
    protected double cmdY = 0.0;
    protected double cmdTurn = 0.0;
    protected static final double JOYSTICK_SLEW = 1;

    // End-effector
    protected DcMotorEx intake = null;
    protected DcMotorEx catapult1 = null;
    protected DcMotorEx catapult2 = null;
    protected Servo catstrength = null;
    protected DcMotorEx lifter = null;

    protected double INTAKE_IN_POWER = -1;
    protected double INTAKE_OFF_POWER = 0.0;

    protected double CATAPULT_UP_POWER = 1;
    protected double CATAPULT_DOWN_POWER = -1;
    protected double CATAPULT_HOLD_DOWN_POWER = 0.0;

    protected double catstrengthPosition = 0.75;
    protected static final double CATSTRENGTH_MIN_POSITION = 0.25;
    protected static final double CATSTRENGTH_MAX_POSITION = 0.75;
    protected static final double CATSTRENGTH_INCREMENT = 0.01;
    protected boolean dpadUpPressed = false;
    protected boolean dpadDownPressed = false;


    @Override
    public void init() {
        // Build alliance-specific pose array
        poseArray = new Pose[poseArrayBlue.length];
        for (int i = 0; i < poseArrayBlue.length; i++) {
            poseArray[i] = (getAlliance() == Alliance.BLUE) ? poseArrayBlue[i] : mirrorBlueToRed(poseArrayBlue[i]);
        }

        follower = Constants.createFollower(hardwareMap);
        if (!demoMode) {
            follower.setStartingPose(PoseStorage.currentPose);
        } else {
            follower.setStartingPose(new Pose(0,0, Math.toRadians(90)));
        }
        follower.update();

        //telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

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

       // Drawing.init();

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

        catstrength = hardwareMap.get(Servo.class, "catstrength");
        catstrengthPosition = 0.75;
        catstrength.setPosition(catstrengthPosition);

        lifter = hardwareMap.get(DcMotorEx.class, "lifter");

        lifter.setDirection(DcMotor.Direction.FORWARD);
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

// Reset encoder
        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifter.setTargetPosition(0);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.setPower(0.7);}

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        updatePoseFromLL();
        //Drawing.drawDebug(follower);

        telemetry.addData("Alliance", getAlliance() == Alliance.BLUE ? "Blue" : "Red");
        //telemetryM.update();

        if (!automatedDrive) {
            double targetY    = gamepad1.right_stick_y * Math.pow(Math.abs(gamepad1.right_stick_y), 1.5);
            double targetX    = gamepad1.right_stick_x * Math.pow(Math.abs(gamepad1.right_stick_x), 1.5);
            double targetTurn = gamepad1.left_stick_x  * Math.pow(Math.abs(gamepad1.left_stick_x),  1.5) / 1.5;

            cmdY    += clamp(targetY    - cmdY,    -JOYSTICK_SLEW, JOYSTICK_SLEW);
            cmdX    += clamp(targetX    - cmdX,    -JOYSTICK_SLEW, JOYSTICK_SLEW);
            cmdTurn += clamp(targetTurn - cmdTurn, -JOYSTICK_SLEW, JOYSTICK_SLEW);

            double mult = slowMode ? slowModeMultiplier : 1.0;
            mult *= driveSpeedCap; // Apply speed cap to all movements

            if (gamepad1.startWasPressed()){
                robotCentric = !robotCentric;

            }
            if (robotCentric){  // Robot Centric
                telemetry.addLine("Robot Centric");

            }
            else{ // Field Centric
                telemetry.addLine("Field Centric");
                if( getAlliance() == Alliance.BLUE ){
                    cmdX = -cmdX;
                    cmdY = -cmdY;
                }
            }
            follower.setTeleOpDrive(
                    -cmdY * mult,
                    -cmdX * mult,
                    -cmdTurn * mult,
                    robotCentric
            );
        }

        // A/B/X/Y automated drive buttons (disabled in demo mode)
        if (!demoMode) {
            handleAutoTarget(gamepad1.aWasPressed(), gamepad1.a, AutoTarget.SCORING);   // A -> scoring
            handleAutoTarget(gamepad1.bWasPressed(), gamepad1.b, AutoTarget.PARKING);   // B -> parking
            handleAutoTarget(gamepad1.xWasPressed(), gamepad1.x, AutoTarget.PICKUP);    // X -> pickup
            handleAutoTarget(gamepad1.yWasPressed(), gamepad1.y, AutoTarget.AUTO_A_END); // Y -> auto A end
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

        // Catstrength servo controls (disabled in demo mode)
        if (!demoMode) {
            // DPad Up - Increase position
            if (gamepad1.dpad_up && !dpadUpPressed) {
                catstrengthPosition += CATSTRENGTH_INCREMENT;
                if (catstrengthPosition > CATSTRENGTH_MAX_POSITION) {
                    catstrengthPosition = CATSTRENGTH_MAX_POSITION;
                }
                catstrength.setPosition(catstrengthPosition);
                dpadUpPressed = true;
            } else if (!gamepad1.dpad_up) {
                dpadUpPressed = false;
            }

            // DPad Down - Decrease position
            if (gamepad1.dpad_down && !dpadDownPressed) {
                catstrengthPosition -= CATSTRENGTH_INCREMENT;
                if (catstrengthPosition < CATSTRENGTH_MIN_POSITION) {
                    catstrengthPosition = CATSTRENGTH_MIN_POSITION;
                }
                catstrength.setPosition(catstrengthPosition);
                dpadDownPressed = true;
            } else if (!gamepad1.dpad_down) {
                dpadDownPressed = false;
            }

            // DPad Right - Increase lifter position
            if (gamepad1.dpad_right) {
                lifterTargetPosition -= 1200;
                ///lifterTargetPosition = upperlimit;
            }

            if (gamepad1.dpad_left) {
                lifterTargetPosition += 1200;
                ///lifterTargetPosition = lowerlimit;
            }

            if (gamepad2.dpad_left) {
                lowerlimit += LIFTER_INCREMENT_TICKS;
                upperlimit += LIFTER_INCREMENT_TICKS;
                lifterTargetPosition += LIFTER_INCREMENT_TICKS;

            }
            lifterTargetPosition = Math.max(lifterTargetPosition,  lowerlimit);
            lifterTargetPosition = Math.min(lifterTargetPosition,  upperlimit);
            lifter.setTargetPosition(lifterTargetPosition);
        }

        Pose odomPose = follower.getPose();
        telemetry.addData("PP X/Y/H", "%4.2f, %4.2f, %4.1f°",
                odomPose.getX(), odomPose.getY(), Math.toDegrees(odomPose.getHeading()));

        telemetry.addData("Catstrength", "%.2f", catstrengthPosition);
        telemetry.addData("Lifter Encoder", lifter.getCurrentPosition());
        telemetry.addData("Lifter Target", lifterTargetPosition);
        telemetry.addData("Lifter Current (mA)",
                lifter.getCurrent(CurrentUnit.MILLIAMPS));
        //telemetryM.debug("position", follower.getPose());
        //telemetryM.debug("velocity", follower.getVelocity());
        //telemetryM.debug("automatedDrive", automatedDrive);
        //telemetryM.debug("autoTarget", currentAutoTarget);
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
            if (gamepad1.back) {
                follower.setPose(llPose);
                telemetry.addData("Set Pose from LL X/Y/H", "%4.2f, %4.2f, %4.1f°",
                        llPose.getX(), llPose.getY(), Math.toDegrees(llPose.getHeading()));
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

    /**
     * Handle automated drive to target when button is pressed/released.
     * @param buttonWasPressed true if button was just pressed
     * @param buttonIsHeld true if button is currently held
     * @param target the AutoTarget to drive to
     */
    protected void handleAutoTarget(boolean buttonWasPressed, boolean buttonIsHeld, AutoTarget target) {
        // Start automated drive when button pressed
        if (buttonWasPressed && !automatedDrive) {
            follower.followPath(pathArray[target.value].get());
            automatedDrive = true;
            currentAutoTarget = target;
        }
        // Stop automated drive when button released
        if (!buttonIsHeld && automatedDrive && currentAutoTarget == target) {
            follower.startTeleopDrive();
            automatedDrive = false;
            currentAutoTarget = AutoTarget.NONE;
        }
    }
}