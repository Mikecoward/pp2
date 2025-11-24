package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Auto Red", group = "Auto")
public class AutoRed extends LinearOpMode {

    // Drive motors
    private DcMotorEx leftFrontDrive;
    private DcMotorEx leftBackDrive;
    private DcMotorEx rightFrontDrive;
    private DcMotorEx rightBackDrive;
    
    private DcMotorEx catapult1 = null;
    private DcMotorEx catapult2 = null;


    // Pinpoint odometry/IMU
    private GoBildaPinpointDriver odo;

    // ---- Encoder constants (adjust for your exact motors/wheels) ----
    // Example: goBILDA 5202/5203 Yellow Jacket, 19.2:1 (about 537.7 ticks/rev)
    private static final double TICKS_PER_REV = 537.7;

    // Example: goBILDA 104mm mecanum wheels
    private static final double WHEEL_DIAMETER_MM = 104.0;
    private static final double MM_PER_REV = Math.PI * WHEEL_DIAMETER_MM;
    private static final double TICKS_PER_MM = TICKS_PER_REV / MM_PER_REV;

    @Override
    public void runOpMode() {

        // ----- Map motors exactly like in TeleOp -----
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "lf");
        leftBackDrive   = hardwareMap.get(DcMotorEx.class, "lb");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rf");
        rightBackDrive  = hardwareMap.get(DcMotorEx.class, "rb");
        
        catapult1 = (DcMotorEx)hardwareMap.get(DcMotor.class, "rcat");
        catapult2 = (DcMotorEx)hardwareMap.get(DcMotor.class, "lcat");

        // Same directions as your TeleOp
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        catapult1.setDirection(DcMotor.Direction.REVERSE); // Backwards should pivot DOWN, or in the stowed position.
        catapult2.setDirection(DcMotor.Direction.FORWARD);
        catapult1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        catapult2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Use encoders
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ----- Set up Pinpoint -----
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        configurePinpoint(true);   // matches your TeleOp settings
        odo.setPosition(new Pose2D(
                DistanceUnit.MM,
                0.0,
                0.0,
                AngleUnit.DEGREES,
                0.0
        ));
        odo.resetPosAndIMU();
        odo.update();

        telemetry.addLine("Auto Initialized - waiting for START");
        telemetry.update();
        
        shoot();

        waitForStart();

        if (!opModeIsActive()) return;
    
        shoot();

        sleep(800);
        
        shoot();
        // 1) Drive backward 50 cm
        driveForwardCm(-50, 0.4);
        
        
        if (!opModeIsActive()) return;

        // 2) Turn 135 degrees left (positive is CCW in our convention)
        turnRelativeDegrees(-135.0, 0.4);
        

        if (!opModeIsActive()) return;

        // 3) Drive forward another 100 cm
        driveForwardCm(50.0, 0.4);

        if (!opModeIsActive()) return;

        // Stop at the end
        setAllDrivePower(0);
        telemetry.addLine("Auto complete");
        telemetry.update();
    }
    

    // ----------------- Motion helpers -----------------

    private void driveForwardCm(double cm, double power) {
        double mm = cm * 10.0;
        int ticksToMove = (int)Math.round(mm * TICKS_PER_MM);

        // Reset encoders
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set targets (same for all four for straight forward)
        leftFrontDrive.setTargetPosition(ticksToMove);
        leftBackDrive.setTargetPosition(ticksToMove);
        rightFrontDrive.setTargetPosition(ticksToMove);
        rightBackDrive.setTargetPosition(ticksToMove);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setAllDrivePower(Math.abs(power));

        // Wait until done or op mode stopped
        while (opModeIsActive()
                && (leftFrontDrive.isBusy() || rightFrontDrive.isBusy()
                || leftBackDrive.isBusy() || rightBackDrive.isBusy())) {

            telemetry.addData("Driving", "Target ticks: %d", ticksToMove);
            telemetry.addData("LF pos", leftFrontDrive.getCurrentPosition());
            telemetry.addData("RF pos", rightFrontDrive.getCurrentPosition());
            telemetry.update();

            idle();
        }

        // Stop and switch back to RUN_USING_ENCODER
        setAllDrivePower(0);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void turnRelativeDegrees(double deltaDegrees, double maxPower) {
        // Get starting heading from Pinpoint
        double startHeading = getRobotHeadingDeg();
        double targetHeading = normalizeAngle(startHeading + deltaDegrees);

        double kP = 0.015;          // P gain for turning (tune as needed)
        double minPower = 0.18;     // minimum power to overcome static friction
        double toleranceDeg = 2.0;  // how close is "good enough"

        while (opModeIsActive()) {
            odo.update();
            double currentHeading = getRobotHeadingDeg();
            double error = angleDiffDeg(targetHeading, currentHeading); // -180..180

            if (Math.abs(error) <= toleranceDeg) {
                break;
            }

            double turnPower = kP * error; // positive error → positive power
            // Clip power
            if (turnPower > 0) {
                turnPower = Math.max(minPower, Math.min(turnPower, maxPower));
            } else {
                turnPower = Math.min(-minPower, Math.max(turnPower, -maxPower));
            }

            // Tank-style turn: left = -turnPower, right = +turnPower
            // (If this turns the wrong direction, swap the sign on turnPower.)
            leftFrontDrive.setPower(-turnPower);
            leftBackDrive.setPower(-turnPower);
            rightFrontDrive.setPower(turnPower);
            rightBackDrive.setPower(turnPower);

            telemetry.addData("Turning", "Target: %.1f, Current: %.1f, Error: %.1f",
                    targetHeading, currentHeading, error);
            telemetry.addData("TurnPower", "%.2f", turnPower);
            telemetry.update();

            idle();
        }

        setAllDrivePower(0);
    }

    private void setAllDrivePower(double p) {
        leftFrontDrive.setPower(p);
        leftBackDrive.setPower(p);
        rightFrontDrive.setPower(p);
        rightBackDrive.setPower(p);
    }

    // ----------------- Pinpoint config & heading helpers -----------------

    private void configurePinpoint(boolean recalibrateIMU) {
        // These settings match your TeleOp's configurePinpoint()

        // Pod offsets (mm)
        odo.setOffsets(-98.0, 150.0, DistanceUnit.MM);

        // Using goBILDA 4-bar odometry pods
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // X pod forward-positive, Y pod left-positive
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        if (recalibrateIMU) {
            odo.recalibrateIMU();
            sleep(500);
            odo.resetPosAndIMU();
            sleep(500);
        }
    }

    /**
     * Returns robot heading in degrees
     */
    private double getRobotHeadingDeg() {
        Pose2D pos = odo.getPosition();
        double rawHeading = pos.getHeading(AngleUnit.DEGREES);
        return normalizeAngle(rawHeading);
    }

    private double normalizeAngle(double angleDeg) {
        while (angleDeg > 180.0) angleDeg -= 360.0;
        while (angleDeg < -180.0) angleDeg += 360.0;
        return angleDeg;
    }

    /** Smallest signed difference target - current in [-180, 180] */
    private double angleDiffDeg(double target, double current) {
        double diff = normalizeAngle(target - current);
        return diff;
    }
    
    private double CATAPULT_UP_POWER = -1;
    private double CATAPULT_DOWN_POWER = 1;
    private double CATAPULT_HOLD_POWER = 0.0;
    private double CATAPULT_HOLD_DOWN_POWER = 0.1;

    
    void shoot() {
        catapult1.setPower(CATAPULT_UP_POWER);
        catapult2.setPower(CATAPULT_UP_POWER);
                sleep(600);
                
                catapult1.setPower(CATAPULT_DOWN_POWER);
                catapult2.setPower(CATAPULT_DOWN_POWER);

                sleep(400);
                catapult1.setPower(CATAPULT_HOLD_DOWN_POWER);
                catapult2.setPower(CATAPULT_HOLD_DOWN_POWER);
    }

}
