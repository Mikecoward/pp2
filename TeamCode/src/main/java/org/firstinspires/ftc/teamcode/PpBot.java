package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLStatus;
import java.util.ArrayList;
import com.qualcomm.robotcore.robot.RobotState;
import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoController;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode2.ColorSensorV31;
//import org.firstinspires.ftc.teamcode2.GoBildaPinpointDriver;

import java.lang.Math;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import java.util.List;
import com.qualcomm.robotcore.hardware.LED;
// stuff for the odometry (imports taken from gpt)
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


import static org.firstinspires.ftc.teamcode.Common.limelight;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/* Control Map

Gamepad A:
Right Stick X: move left/right
Right Stick Y: move forward/back
Left Stick X:  turn robot left/right
Left Stick Y:
Press Right/Left Stick in:

Dpad L/R:
Dpad Up/Down:
A button: increases spinner angle by 60 degrees,
 B button:   push once- hood moves up a little, push again, hood goes down
X button:    while holding, shooter motor is set to max power
Y button: start spinning intake
L button: make kicker go up/down
L trigger:
R button:
R trigger:
Logo (Logitech Button):
back button:
start button:  Set IMU back to 0.

Gamepad 2:
Right Stick X:
Right Stick Y:
Left Stick X:
Left Stick Y:
Dpad L:
Dpad R:
Dpad Up:
Dpad Down:

A button:
B button:
X button:
Y button:
L button:
L trigger:
R button:
R trigger:
Logo (Logitech Button):
back button:
start button:

*/




@TeleOp(name="AS-first robot-3", group="Robot")
//@Disabled

public class PpBot extends LinearOpMode {
    double counter = 0;
    boolean isydone = false;
    boolean isnewydone = false;
    double time_in = 0;
    double starting_time = 0;
    boolean colorf_good;
    boolean colorb_good;

    boolean trial1 = false;
    boolean toggle28 = false;

    double deployDownPos = .765;
    double cycletime = 0;
    double looptime = 0;
    double oldtime = 0;
    boolean toggledispense = true;
    double direction_shifter_1 = 1;



    boolean toggle23 = false;
    boolean toggle24 = false;
    boolean toggle25 = false;
    double counter24 = 0;
    double counter25 = 0;
    double counter23 = 0;
    double counter26 = 0;

    boolean swapDirections = false;
    boolean debounceDirection = false;

    boolean specimenMode = false;
    boolean debounceSpecimen = false;
    IMU imu;
    double shootingpower = 0;


    @Override
    public void runOpMode() {


        char colorf = 'n';
        char colorb = 'n';
        double left;
        double right;
        double forward;
        double rotate;
        double max;
        double speed = 0;
        boolean n_color_override = true;
        double x = 0;
        double y = 0;
        double rx = 0;

        imu = hardwareMap.get(IMU.class, "imu");

        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);


        Common.telemetry = telemetry;
        if (Common.initialPositionSet) {
            Common.configRobot(hardwareMap, false);
            telemetry.addLine("Taking position from previous run");

        } else {
            Common.configRobot(hardwareMap, true);
            Common.setInitialPosition(-31.5, -63.5, 0);  // Right edge of robot aligned with second tile seam from right
            telemetry.addLine("Setting position to -31.5, -63.5, 0");
        }

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();

        telemetry.addLine("Robot Ready.");
        telemetry.update();

        /* Wait for the game driver to press play */
        waitForStart();
/*
        Common.updatePinpoint();

        Common.zeroBothMotors();
        /*
        /* Run until the driver presses stop */
        boolean lastA = false;
        float rightTrigger = 0;
        float leftTrigger = 0;
        boolean rightbumperpressed = false;
        boolean leftbumperpressed = false;

        double curAngle = 62; // current angle
        //Common.Spinner.setPosition(curAngle/360);

        //boolean hoodUp = false; // starts down
        boolean kickerUp = false;
        long lastKickTime = 0;
        Pose2D pos = Common.ppPos;

        // At top of OpMode
        boolean lastB = false;
        boolean hoodUp = false;

        double moveAmount = 24 * 360;      // degrees per press (2 rotations)
        double incrementAmount = 60; // degrees per press (2 rotations)

        String[] ballposition = {"na", "na", "na"};
        int curballselected = 0;
        double kP = 0.01;

        // SPINNER STUFF
        double spinnerError = 0;
        double spinnerSpeed = 0;
        double last_spinnerError = 0;
        double targetRotation = 0;


        double targetAngle = 0.0;

        boolean intaking = false;
        boolean shooting = false;
        boolean ball_shoot_selected = false;
        boolean kickerready = false;
        String ballshoot = "na";


        final double TURN_SPEED = 0.25; // Slower speed for alignment
        final double TX_DEADBAND = 2.0; // Target is "aligned" if tx is within +/- 1.0 degrees

        while (opModeIsActive()) {

            LLStatus status = limelight.getStatus();
            LLResult result = limelight.getLatestResult();

            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
            }
            Common.updatePinpoint();


            Common.spin.update();
            looptime = System.currentTimeMillis() - oldtime;
            oldtime = System.currentTimeMillis();

            // Calculate the error
            double currentRotation = Common.spin.getTotalRotation();
            spinnerError = targetRotation - currentRotation; // Correct error calculation (Target - Current)
            spinnerSpeed = (spinnerError - last_spinnerError) / looptime; // Calculate speed
            last_spinnerError = spinnerError;

            double power = -spinnerError * 0.005 + spinnerSpeed * 0.0;
            power = Math.min(power, 0.2);
            power = Math.max(power, -0.2);

            // Set a "deadband" or tolerance. If the error is very small, just stop.
            if (Math.abs(spinnerError) > 1) {
                // Move until target reached
                Common.spin.setPower(power);
            } else {
                // When we are close enough to the target, stop the motor.
                Common.spin.setPower(0);
            }


            telemetry.addData("spinner angle", currentRotation);
            telemetry.addData("target spinner angle", targetRotation);
            telemetry.addData("error", spinnerError);
            telemetry.addData("power", power);


            if (gamepad2.start) {
                Common.zeroBothMotors();
            }

            if (gamepad1.b && !lastB) {
                if (!hoodUp) {
                    targetAngle += moveAmount; // move "up" 2 rotations
                    hoodUp = true;
                } else {
                    targetAngle -= moveAmount; // move "down" 2 rotations
                    hoodUp = false;
                }
            }




/*
            if (gamepad1.left_bumper && !leftbumperpressed) {
                if  (targetRotation%60 == 0) {
                    Common.kicker.setPosition(0.18);  // move up
                    kickerUp = true;
                    leftbumperpressed = true;
                    lastKickTime = System.currentTimeMillis(); // record time
                }
            }


 */

            if (!gamepad1.left_bumper) {
                leftbumperpressed = false;
            }


            if (gamepad1.x) {
                ((DcMotorEx) Common.shooterMotor).setVelocity(-6000 / 60.0 * 28.0);

            } else {
                ((DcMotorEx) Common.shooterMotor).setVelocity(0.0);

            }


            //=========================================================
            // INDEPENDENT INTAKE CONTROL (left bumper)
            //=========================================================
            if (leftbumperpressed && gamepad1.left_bumper && (ballposition[0].equals("na") || ballposition[1].equals("na") || ballposition[2].equals("na"))) {
                // When Y is pressed, turn the intake motors on.
                intaking = true;
                if (targetRotation % 360 == 60 || targetRotation % 360 == 180 || targetRotation % 360 == 300) {
                    targetRotation += 60;
                    curballselected = (curballselected + 1) % 3;
                    ;
                }

                while (!ballposition[curballselected].equals("na")) {
                    targetRotation += 2 * incrementAmount;
                    curballselected = (curballselected + 1) % 3;

                }
            }


            if (intaking) {
                Common.rightIntake.setPower(1);
                Common.leftIntake.setPower(1);
                Common.upperIntake.setPower(1);
            } else {
                // When Y is NOT pressed, turn the intake motors off.
                Common.rightIntake.setPower(0);
                Common.leftIntake.setPower(0);
                Common.upperIntake.setPower(0);
            }

            if (gamepad1.right_bumper && intaking) {
                ballposition[curballselected] = "green";
                intaking = false;
                targetRotation += incrementAmount;
            } else if (gamepad1.right_bumper && shooting) {
                ball_shoot_selected = true;
                ballshoot = "green";

            }
            if (gamepad1.right_trigger > 0.3 && intaking && !ball_shoot_selected) {
                ballposition[curballselected] = "purple";
                intaking = false;
                targetRotation += incrementAmount;
            } else if (gamepad1.right_trigger > 0.3 && shooting && !ball_shoot_selected) {
                ball_shoot_selected = true;
                ballshoot = "purple";
            }

            if ((!ballposition[0].equals(ballshoot) && !ballposition[1].equals(ballshoot) && !ballposition[2].equals(ballshoot)) && !ballshoot.equals("na")) {
                shooting = false;
                ballshoot = "na";
                ball_shoot_selected = false;
            }

            telemetry.addData("ballposition", ballposition[0] + " " + ballposition[1] + " " + ballposition[2]);


            //=========================================================
            // INDEPENDENT shoot CONTROL (left trigger)
            //=========================================================

            if (leftTrigger == 0 && gamepad1.left_trigger >= 0.1 && !intaking) {
                // When Y is pressed, turn the intake motors on.
                ball_shoot_selected = false;
                shooting = true;
                kickerready = false;
                kickerUp = false;
            }


            telemetry.addData("shooting", shooting);
            telemetry.addData("ball_shoot_selected", ball_shoot_selected);
            telemetry.addData("ballshoot", ballshoot);


            telemetry.addData("ballposition", ballposition[0] + " " + ballposition[1] + " " + ballposition[2]);
            telemetry.addData("curballselected", curballselected);
            telemetry.addData("kickerready", kickerready);
            telemetry.addData("kickerUp", kickerUp);

            if (ball_shoot_selected && !ballshoot.equals("na")) {

                telemetry.addLine("____ SHOOTING ____");
                if (targetRotation % 360 == 0 || targetRotation % 360 == 120 || targetRotation % 360 == 240) {
                    targetRotation += incrementAmount;
                    curballselected = (curballselected + 1) % 3;
                }

                telemetry.addLine("one has it");
                if (ballposition[(curballselected) % 3].equals(ballshoot)) {
                    targetRotation += 2 * incrementAmount;
                    curballselected = (curballselected + 1) % 3;
                    telemetry.addLine("____  add____");
                } else if (ballposition[(curballselected + 2) % 3].equals(ballshoot)) {
                    // do nothing
                    telemetry.addLine("____ nothing ____");

                } else if (ballposition[(curballselected + 1) % 3].equals(ballshoot)) {
                    telemetry.addLine("____ subtract ____");
                    targetRotation += 4 * incrementAmount;
                    curballselected = (curballselected + 2) % 3;
                }


                shootingpower = -1400;
                ball_shoot_selected = false;
                kickerready = true;
            }//1

            telemetry.addData("shootingpower", shootingpower);
            ((DcMotorEx) Common.shooterMotor).setVelocity(shootingpower);

            if (((DcMotorEx) Common.shooterMotor).getVelocity() <= 1400 && shooting && kickerready && shootingpower != 0 ) {
                telemetry.addLine("shooting");
                Common.kicker.setPosition(0.18);  // move up
                lastKickTime = System.currentTimeMillis(); // record time
                kickerUp = true;

            }
            telemetry.addData("time", System.currentTimeMillis() - lastKickTime);
            if (System.currentTimeMillis() - lastKickTime > 500 && shooting && kickerUp) { // after 0.5s
                telemetry.addLine("kickerdown");

                Common.kicker.setPosition(0.655); // move down
                ballposition[(curballselected + 1) % 3] = "na";
                ballshoot = "na";
                ball_shoot_selected = false;
                shooting = false;
                kickerUp = false;
                kickerready = false;
                shootingpower = 0;
            }

            lastB = gamepad1.b;
            leftbumperpressed = gamepad1.left_bumper;
            leftTrigger = gamepad1.left_trigger;
            leftTrigger = gamepad1.left_trigger;
            rightTrigger = gamepad1.right_trigger;
            rightbumperpressed = gamepad1.right_bumper;


            telemetry.addData("Servo Angle", curAngle);

            telemetry.addData("motor speed", ((DcMotorEx) Common.shooterMotor).getVelocity());
            if (gamepad1.a) {
                telemetry.addLine("Pressed A");
            }
            if (gamepad1.b) {
                telemetry.addLine("Pressed B");
            }
            if (gamepad1.x) {
                telemetry.addLine("Pressed X");
            }
            if (gamepad1.y) {
                telemetry.addLine("Pressed Y");
            }

            if (gamepad1.start) {
                Common.setInitialPosition(16.5, -63.5, 0);  // Right edge of robot aligned with second tile seam from right
            }

            handleJoystick();
            telemetry.update();
        }
    }

    private double applyExpo(double input, double powerfactor, double minfactor) {
        if (input == 0) return 0;
        return input * Math.pow(Math.abs(input), powerfactor - 1)
                + (Math.signum(input) * minfactor);
    }

    private void handleJoystick() {
        // Handle Joysticks


        double stickY = -gamepad1.right_stick_y;
        double stickX = gamepad1.right_stick_x;
        double stickR = gamepad1.left_stick_x;

        // Reverse Sticks

        if (swapDirections) {
            stickY = -stickY;
            stickX = -stickX;
        }
        double minfactor = 0.05;
        double powerfactor = 1.200;
        // Expo control:
        if (!(stickX == 0)) {
            stickX = stickX * Math.pow(Math.abs(stickX), powerfactor-1)+ (stickX/Math.abs(stickX)*minfactor);
        }
        if (!(stickY == 0)) {
            stickY = stickY * Math.pow(Math.abs(stickY), powerfactor-1)+ (stickY/Math.abs(stickY)*minfactor);
        }
        if (!(stickR == 0)) {
            stickR = stickR * Math.pow(Math.abs(stickR), powerfactor-1)+ (stickR/Math.abs(stickR)*minfactor);
        }
        stickX = stickX * 1.1; // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        boolean headingfield = true;
        double frontLeftPower;
        double backLeftPower;
        double frontRightPower;
        double backRightPower;

        if (headingfield){

            double rotX = stickX * Math.cos(-botHeading) - stickY * Math.sin(-botHeading);
            double rotY = stickX * Math.sin(-botHeading) + stickY * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            frontLeftPower = (rotY + rotX + stickR);
            backLeftPower = (rotY - rotX + stickR);
            frontRightPower = (rotY - rotX - stickR);
            backRightPower = (rotY + rotX - stickR);

        }
        else{
            frontLeftPower = (stickY + stickX + stickR);
            backLeftPower = (stickY - stickX + stickR);
            frontRightPower = (stickY - stickX - stickR);
            backRightPower = (stickY + stickX - stickR);

        }

        // If going backward, do it slower
        //if (stickY<0) stickY /= 2;

        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower)),
                Math.max(Math.abs(frontRightPower), Math.abs(backRightPower)));
        if (maxPower>1.0) {
            frontLeftPower /= maxPower;
            backLeftPower /= maxPower;
            frontRightPower/= maxPower;
            backRightPower/= maxPower;
        }

        //these codes just set the power for everything
        ((DcMotorEx) Common.leftFrontDrive).setPower(frontLeftPower);
        ((DcMotorEx) Common.leftBackDrive).setPower(backLeftPower);
        ((DcMotorEx) Common.rightFrontDrive).setPower(frontRightPower);
        ((DcMotorEx) Common.rightBackDrive).setPower(backRightPower);
    }
}