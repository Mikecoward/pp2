//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.IMU;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//
//// stuff for the odometry (imports taken from gpt)
//
//
//
//
//
///* Control Map
//
//Gamepad A:
//Right Stick X: move left/right
//Right Stick Y: move forward/back
//Left Stick X:  turn robot left/right
//Left Stick Y:
//Press Right/Left Stick in:
//
//Dpad L/R:
//Dpad Up/Down:
//A button: increases spinner angle by 60 degrees,
// B button:   push once- hood moves up a little, push again, hood goes down
//X button:    while holding, shooter motor is set to max power
//Y button: start spinning intake
//L button: make kicker go up/down
//L trigger:
//R button:
//R trigger:
//Logo (Logitech Button):
//back button:
//start button:  Set IMU back to 0.
//
//Gamepad 2:
//Right Stick X:
//Right Stick Y:
//Left Stick X:
//Left Stick Y:
//Dpad L:
//Dpad R:
//Dpad Up:
//Dpad Down:
//
//A button:
//B button:
//X button:
//Y button:
//L button:
//L trigger:
//R button:
//R trigger:
//Logo (Logitech Button):
//back button:
//start button:
//
//*/
//
//
//@TeleOp(name="VikraM-variation", group="Robot")
////@Disabled
//
//public class VikramPpBot extends LinearOpMode {
//    double counter = 0;
//    boolean isydone = false;
//    boolean isnewydone = false;
//    double time_in = 0;
//    double starting_time = 0;
//    boolean colorf_good;
//    boolean colorb_good;
//
//    boolean trial1 = false;
//    boolean toggle28 = false;
//
//    double deployDownPos = .765;
//    double cycletime = 0;
//    double looptime = 0;
//    double oldtime = 0;
//    boolean toggledispense = true;
//    double direction_shifter_1 = 1;
//
//    boolean toggle23 = false;
//    boolean toggle24 = false;
//    boolean toggle25 = false;
//    double counter24 = 0;
//    double counter25 = 0;
//    double counter23 = 0;
//    double counter26 = 0;
//
//    boolean swapDirections = false;
//    boolean debounceDirection = false;
//
//    boolean specimenMode = false;
//    boolean debounceSpecimen = false;
//    IMU imu;
//
//
//
//
//    @Override
//    public void runOpMode() {
//        /*
//        These variables are private to the OpMode, and are used to control the drivetrain.
//         */
//
//        char colorf = 'n';
//        char colorb = 'n';
//        double left;
//        double right;
//        double forward;
//        double rotate;
//        double max;
//        double speed = 0;
//        boolean n_color_override = true;
//        double x = 0;
//        double y = 0;
//        double rx = 0;
//
//        imu = hardwareMap.get(IMU.class, "imu");
//
//        // Adjust the orientation parameters to match your robot
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//            RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
//            RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
//
//        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
//        imu.initialize(parameters);
//
//
//        Common.telemetry = telemetry;
//        if (Common.initialPositionSet) {
//            Common.configRobot(hardwareMap, false);
//            telemetry.addLine("Taking position from previous run");
//
//        } else {
//            Common.configRobot(hardwareMap, true);
//            Common.setInitialPosition(-31.5, -63.5, 0);  // Right edge of robot aligned with second tile seam from right
//            telemetry.addLine("Setting position to -31.5, -63.5, 0");
//        }
//
//        telemetry.addLine("Robot Ready.");
//        telemetry.update();
//
//        /* Wait for the game driver to press play */
//        waitForStart();
///*
//        Common.updatePinpoint();
//
//        Common.zeroBothMotors();
//        /*
//        /* Run until the driver presses stop */
//        boolean lastA = false; // tracks previous state of button
//        boolean lastB = false;
//        boolean leftbumperpressed = false;
//        double curAngle = 27; // current angle
//        Common.Spinner.setPosition(curAngle/360);
//
//        boolean hoodUp = false; // starts down
//        boolean kickerUp = false;
//        double moveAmount = 0.2;
//        double moveAmountKick = 0.2;
//
//        while (opModeIsActive()){
//
//            Common.updatePinpoint();   // <-- Step 1, update Pinpoint
//            telemetry.update();        // push telemetry to DS screen
//
//            /*telemetry.addData("rangeL", String.format("%.01f mm", Common.sensorDistanceL.getDistance(DistanceUnit.MM)));
//            telemetry.addData("rangeR", String.format("%.01f mm", Common.sensorDistanceR.getDistance(DistanceUnit.MM)));
//            */
//            if (gamepad2.start){
//                Common.zeroBothMotors();
//            }
//
//            // Button pressed now, but wasn't pressed last loop
//            if (gamepad1.a && !lastA && !kickerUp) {
//                curAngle += 69.0;
//                if (curAngle > 360) curAngle = 27; // wrap around
//                Common.Spinner.setPosition(curAngle/360.0);
//            }
//            /*
//            curAngle += 0.3*gamepad1.left_trigger;
//            curAngle -= 0.3*gamepad1.right_trigger;
//            Common.Spinner.setPosition(curAngle/360.0);
//            */
//            if (gamepad1.b && !lastB) {
//                double pos = Common.AngleHood.getPosition();
//
//                if (!hoodUp) {
//                    pos += moveAmount;  // move up
//                    hoodUp = true;
//                } else {
//                    pos -= moveAmount;  // move down
//                    hoodUp = false;
//                }
//
//                // clamp to 0–1
//                if (pos > 1) pos = 1;
//                if (pos < 0) pos = 0;
//
//                Common.AngleHood.setPosition(pos);
//            }
//            if (gamepad1.left_bumper && !leftbumperpressed) {
//                double kickerPosition = Common.kicker.getPosition();
//                if(curAngle !=96  && curAngle != 234) {
//                    if (!kickerUp) {
//                        kickerPosition = .18;  // move up
//                        kickerUp = true;
//                    } else {
//                        kickerPosition = .655;  // move down
//                        kickerUp = false;
//                    }
//                }
//                // clamp to 0–1
//                if (kickerPosition > 1) kickerPosition = 1;
//                if (kickerPosition < 0) kickerPosition = 0;
//                leftbumperpressed = true;
//                Common.kicker.setPosition(kickerPosition);
//            }
//            if(!gamepad1.left_bumper){
//                leftbumperpressed = false;
//            }
//
//            if (gamepad1.x){
//                Common.shooterMotor.setPower(1);
//            }
//            else {
//                Common.shooterMotor.setPower(0);
//            }
//            if (gamepad1.y){
//                Common.rightIntake.setPower(1);
//                Common.leftIntake.setPower(1);
//            }
//            else {
//                Common.rightIntake.setPower(0);
//                Common.leftIntake.setPower(0);
//            }
//            lastA = gamepad1.a;
//            lastB = gamepad1.b;
//            lastA = gamepad1.a; // update button state
//            telemetry.addData("Servo Angle", curAngle);
//
//            if (gamepad1.a) {
//                telemetry.addLine("Pressed A");
//            }
//            if (gamepad1.b) {
//                telemetry.addLine("Pressed B");
//            }
//            if (gamepad1.x) {
//                telemetry.addLine("Pressed X");
//            }
//            if (gamepad1.y) {
//                telemetry.addLine("Pressed Y");
//            }
//
//            if (gamepad1.start) {
//                Common.setInitialPosition(16.5, -63.5, 0);  // Right edge of robot aligned with second tile seam from right
//            }
//
//            handleJoystick();
//            telemetry.update();
//        }
//    }
//
//    private void handleJoystick() {
//    // Handle Joysticks
//
//
//
//    double stickY = -gamepad1.right_stick_y;
//    double stickX = gamepad1.right_stick_x;
//    double stickR = gamepad1.left_stick_x;
//
//    // Reverse Sticks
//
//    if (swapDirections) {
//        stickY = -stickY;
//        stickX = -stickX;
//    }
//    double minfactor = 0.15;
//    double powerfactor = .20;
//    // Expo control:
//    if (!(stickX == 0)) {
//        stickX = stickX * Math.pow(Math.abs(stickX), powerfactor-1)+ (stickX/Math.abs(stickX)*minfactor);
//    }
//    if (!(stickY == 0)) {
//        stickY = stickY * Math.pow(Math.abs(stickY), powerfactor-1)+ (stickY/Math.abs(stickY)*minfactor);
//    }
//    if (!(stickR == 0)) {
//        stickR = stickR * Math.pow(Math.abs(stickR), powerfactor-1)+ (stickR/Math.abs(stickR)*minfactor);
//    }
//    stickX = stickX * 1.1; // Counteract imperfect strafing
//
//    // Denominator is the largest motor power (absolute value) or 1
//    // This ensures all the powers maintain the same ratio,
//    // but only if at least one is out of the range [-1, 1]
//    double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//
//    boolean headingfield = true;
//    double frontLeftPower;
//    double backLeftPower;
//    double frontRightPower;
//    double backRightPower;
//
//    if (headingfield){
//
//        double rotX = stickX * Math.cos(-botHeading) - stickY * Math.sin(-botHeading);
//        double rotY = stickX * Math.sin(-botHeading) + stickY * Math.cos(-botHeading);
//
//       rotX = rotX * 1.1;  // Counteract imperfect strafing
//
//       // Denominator is the largest motor power (absolute value) or 1
//       // This ensures all the powers maintain the same ratio,
//       // but only if at least one is out of the range [-1, 1]
//        frontLeftPower = (rotY + rotX + stickR);
//        backLeftPower = (rotY - rotX + stickR);
//        frontRightPower = (rotY - rotX - stickR);
//        backRightPower = (rotY + rotX - stickR);
//
//    }
//    else{
//         frontLeftPower = (stickY + stickX + stickR);
//         backLeftPower = (stickY - stickX + stickR);
//         frontRightPower = (stickY - stickX - stickR);
//         backRightPower = (stickY + stickX - stickR);
//
//    }
//
//    // If going backward, do it slower
//    //if (stickY<0) stickY /= 2;
//
//    double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower)),
//    Math.max(Math.abs(frontRightPower), Math.abs(backRightPower)));
//    if (maxPower>1.0) {
//        frontLeftPower /= maxPower;
//        backLeftPower /= maxPower;
//        frontRightPower/= maxPower;
//        backRightPower/= maxPower;
//    }
//
//    //these codes just set the power for everything
//    ((DcMotorEx) Common.leftFrontDrive).setPower(frontLeftPower);
//    ((DcMotorEx) Common.leftBackDrive).setPower(backLeftPower);
//    ((DcMotorEx) Common.rightFrontDrive).setPower(frontRightPower);
//    ((DcMotorEx) Common.rightBackDrive).setPower(backRightPower);
//}
//
//}
