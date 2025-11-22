package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;
import java.util.Locale;


class VikramCommon {
    static DcMotor  leftFrontDrive   = null;
    static DcMotor  rightFrontDrive  = null;
    static DcMotor  leftBackDrive    = null;
    static DcMotor  rightBackDrive   = null;
    static DcMotor  shooterMotor      = null;
    static CRServo leftIntake = null;
    static CRServo rightIntake = null;
    public static Servo kicker = null;



    static Servo IntakeDeploy = null;
    static Servo bucketServo = null;
    static Servo sweeperServo = null;
    static DcMotor  liftMotor        = null;
    static DcMotor  armExtMotor      = null;

    static CRServo intakeServo1 = null;
    static CRServo intakeServo0 = null;

    static Limelight3A limelight;
    static IMU imu = null;

    static LED topLED_red;
    static LED topLED_green;

    static boolean redAlliance = false;

    static double INTAKE_UP = 0.07;
    static double INTAKE_DUMP_SPECIMEN = 0.55;
    static double INTAKE_DOWN = 0.73;

    static double BUCKET_UP = 0.26;
    static double BUCKET_DOWN = 0.91;

    static double SWEEPER_IN = 0.39;
    static double SWEEPER_OUT = 0.8;

    static GoBildaPinpointDriver odo;
    static Pose2D ppPos, ppVel, targetPos;
    static int ppPosAbsolute = 0;  // Number of updates.  0 is relative, 1+ is absolute
    static int ppPosThreshV = 0;    // Velocity blocked
    static int ppPosThreshS = 0;    // Fiducial size blocked


    static Telemetry telemetry = null;
    static Gamepad gamepad1 = null;
    static Gamepad gamepad2 = null;


    static double liftPosition = 0;
    static double armExtPosition = 0;

//    static ColorSensorV31 colorSensorF;
//    static ColorSensorV31 colorSensorB;

    static DistanceSensor sensorDistanceL;
    static DistanceSensor sensorDistanceR;

    public static Servo AngleHood;
    public static Servo Spinner;

    // Run motor slowly downwards until current gets too high, then decide that this must be the zero point.
    static void zeroBothMotors() {
        bucketServo.setPosition(BUCKET_DOWN);
        IntakeDeploy.setPosition(INTAKE_UP);

        armExtMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armExtMotor.setPower(-0.4);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setPower(-0.4);
        boolean endext = false;
        boolean endlift = false;
        while (!endext|| !endlift) {
            telemetry.addData("Arm Motor Current:",((DcMotorEx) armExtMotor).getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Lift Motor Current:",((DcMotorEx) liftMotor).getCurrent(CurrentUnit.AMPS));

            if (((DcMotorEx) liftMotor).getCurrent(CurrentUnit.AMPS)>2.0){
                liftMotor.setTargetPosition(0);
                liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                ((DcMotorEx) liftMotor).setVelocity(2300);
                // Get off the bottom slightly to avoid overcurrent
                liftMotor.setTargetPosition(20);
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotor.setTargetPosition(20);
                liftPosition = 20;
                endlift = true;
            }
            if (((DcMotorEx) armExtMotor).getCurrent(CurrentUnit.AMPS)>2.0){
                armExtMotor.setTargetPosition(0);
                armExtMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                ((DcMotorEx) armExtMotor).setVelocity(2300);


                // Get off the bottom slightly to avoid overcurrent
                armExtMotor.setTargetPosition(40);
                armExtMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armExtMotor.setTargetPosition(40);
                armExtPosition = 40;
                endext = true;
            }
            telemetry.update();
        }
    }

    static void configRobot(HardwareMap hardwareMap, boolean recalibrateIMU) {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Set pose to X=0 mm, Y=0 mm, Heading=0 degrees
        odo.setPosition(new Pose2D(
                DistanceUnit.MM,
                0.0,
                0.0,
                AngleUnit.DEGREES,
                0.0
        ));

        odo.resetPosAndIMU();

        AngleHood = hardwareMap.get(Servo.class, "hood");
        Spinner = hardwareMap.get(Servo.class, "spinner");
        kicker = hardwareMap.get(Servo.class, "kicker");

        /*configurePinpoint(hardwareMap, recalibrateIMU);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

        colorSensorF = hardwareMap.get(ColorSensorV31.class, "sensor_color");
        colorSensorF.initSensor();
        colorSensorB = hardwareMap.get(ColorSensorV31.class, "sensor_color2");
        colorSensorB.initSensor();


        sensorDistanceL = hardwareMap.get(DistanceSensor.class, "sensor_distance0");
        sensorDistanceR = hardwareMap.get(DistanceSensor.class, "sensor_distance1");
        */
        /* Define and Initialize Motors */
        leftFrontDrive  = hardwareMap.dcMotor.get("leftFront");
        leftBackDrive   = hardwareMap.dcMotor.get("leftBack");
        rightFrontDrive = hardwareMap.dcMotor.get("rightFront");
        rightBackDrive  = hardwareMap.dcMotor.get("rightBack");
        shooterMotor  = hardwareMap.dcMotor.get("shooterMotor");
        leftIntake = hardwareMap.get(CRServo.class, "leftIntake");
        rightIntake = hardwareMap.get(CRServo.class, "rightIntake");

        /*
        liftMotor       = hardwareMap.dcMotor.get("vertmotor");
        //armRotMotor     = hardwareMap.dcMotor.get("armrotmotor");
        armExtMotor     = hardwareMap.dcMotor.get("armextmotor");

       */
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntake.setDirection(CRServo.Direction.REVERSE);
        /*
         Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to slow down
        much faster when it is coasting. This creates a much more controllable drivetrain. As the robot
        stops much quicker. */
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        /*
        ((DcMotorEx) liftMotor).setCurrentAlert(2,CurrentUnit.AMPS);

        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ((DcMotorEx) liftMotor).setVelocity(2300);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        armExtMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armExtMotor.setTargetPosition(0);
        armExtMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armExtMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ((DcMotorEx) armExtMotor).setVelocity(2300);
        armExtMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intakeServo1 = hardwareMap.get(CRServo.class, "intake1");
        intakeServo0 = hardwareMap.get(CRServo.class, "intake0");
        IntakeDeploy = hardwareMap.get(Servo.class, "intakeDeploy");
        bucketServo = hardwareMap.get(Servo.class, "bucket");
        sweeperServo = hardwareMap.get(Servo.class, "sweeper");
        sleep(100);
        sweeperServo.setPosition(SWEEPER_IN);
        */
        checkAlliance(hardwareMap);
        /*
        topLED_green = hardwareMap.get(LED.class, "led1");
        topLED_red = hardwareMap.get(LED.class, "led0");
        topLED_green.off();
        topLED_red.off();*/

    }

    static boolean initialPositionSet = false;
    // Sets position in the Pinpoint IMU
    static void setInitialPosition(double X, double Y, double H) {
        /*odo.setPosition(new Pose2D(DistanceUnit.INCH,
                X,
                Y,
                AngleUnit.DEGREES,
                H+90));  // Add 90 to correct for field vs Pinpoint coordinates
        initialPositionSet = true;
    }

    static void tempMT2() {
         // Temporarily set ppPos so limelight doesn't crash
        ppPos = new Pose2D(DistanceUnit.MM,
                0,
                0,
                AngleUnit.DEGREES,
                90);

        checkLimelight();

        if (isMt2Valid()) {
            telemetry.addLine("Valid");
            odo.setPosition(new Pose2D(DistanceUnit.INCH,
                getMt2X(),
                getMt2Y(),
                AngleUnit.DEGREES,
                90));
            telemetry.addLine(getMt2X()+", "+getMt2Y());
            ppPosAbsolute++;
            //updatePinpoint();

        } else {
            telemetry.addLine("not Valid");
            odo.setPosition(new Pose2D(DistanceUnit.MM,
                0,
                0,
                AngleUnit.DEGREES,
                90));
        }
    }

    static void sweepField() {
        sweeperServo.setPosition(SWEEPER_OUT);
        sleep(1000);
        sweeperServo.setPosition(SWEEPER_IN);
    }

    static void updatePinpoint() {
            odo.update();

            ppPos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f, A: %d, V: %d, F: %d}", ppPos.getX(DistanceUnit.INCH), ppPos.getY(DistanceUnit.INCH), normalizeAngleD(ppPos.getHeading(AngleUnit.DEGREES)-90), ppPosAbsolute, ppPosThreshV, ppPosThreshS);
            telemetry.addData("Position", data);


            gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.

            ppVel = odo.getVelocity();
            String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", ppVel.getX(DistanceUnit.MM), ppVel.getY(DistanceUnit.MM), ppVel.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Velocity", velocity);


            Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll primarily see
            READY: the device is working as normal
            CALIBRATING: the device is calibrating and outputs are put on hold
            NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
            FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
            FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
            FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in

            telemetry.addData("Status", odo.getDeviceStatus());
    */

    }

    static void configurePinpoint(HardwareMap hardwareMap, boolean recalibrateIMU) {
        /* // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");

*/
        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        //odo.setOffsets(-85.0, 126.0);

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per mm of your odometry pod.
         */
       // odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        //odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);


        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        /*
        if (recalibrateIMU) {
            odo.recalibrateIMU();
            sleep(500);
            odo.resetPosAndIMU();
            sleep(500);
        }
        */
    }

    static boolean isMt2Valid() {
        return botpose_mt2!=null;
    }

    static void updatePinpoint() {
        odo.update();

        ppPos = odo.getPosition();
        String data = String.format(Locale.US,
                "{X: %.3f, Y: %.3f, H: %.3f, A: %d, V: %d, F: %d}",
                ppPos.getX(DistanceUnit.INCH),
                ppPos.getY(DistanceUnit.INCH),

                normalizeAngleD(ppPos.getHeading(AngleUnit.DEGREES)-90),
                ppPosAbsolute, ppPosThreshV, ppPosThreshS);
        telemetry.addData("Position", data);

        Log.d("Pinpoint", String.format("{X: %.3f, Y: %.3f, H: %.3f}",
                ppPos.getX(DistanceUnit.INCH),
                ppPos.getY(DistanceUnit.INCH),
                normalizeAngleD(ppPos.getHeading(AngleUnit.DEGREES)-90)
        ));
        /*
        ppVel = odo.getVelocity();
        String velocity = String.format(Locale.US,
                "{XVel: %.3f, YVel: %.3f, HVel: %.3f}",
                ppVel.getX(DistanceUnit.MM),
                ppVel.getY(DistanceUnit.MM),
                ppVel.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Velocity", velocity);

        telemetry.addData("Status", odo.getDeviceStatus());

         */
    }

    // returns these in inches
    static double getMt2X() {
        return botpose_mt2.getPosition().x/.0254;
    }

    static double getMt2Y() {
        return botpose_mt2.getPosition().y/.0254;
    }

    static Pose3D botpose_mt2;

    static void checkLimelight() {
        double FIDUCIAL_THRESHOLD_MT2 = 0.0035;

        boolean mt2_valid = false;

        LLStatus status = limelight.getStatus();
        /*telemetry.addData("Name", "%s",
                status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(),(int)status.getFps());f
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());
           */
            // First, tell Limelight which way your robot is facing
            double robotYaw = ppPos.getHeading(AngleUnit.DEGREES);

            if (!redAlliance) {
                robotYaw = normalizeAngleD(robotYaw+180);
            }

            //imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - initialYaw + llYawOffset;

            limelight.updateRobotOrientation(robotYaw);

            LLResult result = limelight.getLatestResult();
            botpose_mt2 = null;

            if (result != null && result.isValid()) {
                            }

            if (result != null) {

                if (result.isValid()) {
                    // Access fiducial results
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f, ta: %.4f", fr.getFiducialId(), fr.getFamily(),fr.getTargetXDegrees(), fr.getTargetYDegrees(), fr.getTargetArea());
                        if (fr.getTargetArea()>FIDUCIAL_THRESHOLD_MT2) {
                            mt2_valid = true;
                        } else {
                            ppPosThreshS++;
                        }
                    }


                    if (mt2_valid) { // If we're close enough to ficucial to trust it:
                        botpose_mt2 = result.getBotpose_MT2();
                        if (botpose_mt2 != null) {
                            if (!redAlliance) { // Flip coordinates if on blue team
                                Position flippedPose = new Position(botpose_mt2.getPosition().unit,-botpose_mt2.getPosition().x, -botpose_mt2.getPosition().y, botpose_mt2.getPosition().z, botpose_mt2.getPosition().acquisitionTime);
                                botpose_mt2 = new Pose3D(flippedPose, botpose_mt2.getOrientation());
                            }

                            telemetry.addData("MT2 Location:", "(" + botpose_mt2.getPosition().x/.0254 + "in, " + botpose_mt2.getPosition().y/.0254 + "in)");

                            telemetry.addData("Yaw", robotYaw);
                        }
                    }

                    if (result.isValid()) {
                        Pose3D botpose = result.getBotpose();
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();
                telemetry.addData("LL Latency", captureLatency + targetingLatency);
                telemetry.addData("Parse Latency", parseLatency);
                telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("txnc", result.getTxNC());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("tync", result.getTyNC());

                    telemetry.addData("Botpose", botpose.toString());

                    // Access barcode results
                    List<LLResultTypes.BarcodeResult> barcodeResults = result.getBarcodeResults();
                    for (LLResultTypes.BarcodeResult br : barcodeResults) {
                        telemetry.addData("Barcode", "Data: %s", br.getData());
                    }

                    // Access classifier results
                    List<LLResultTypes.ClassifierResult> classifierResults = result.getClassifierResults();
                    for (LLResultTypes.ClassifierResult cr : classifierResults) {
                        telemetry.addData("Classifier", "Class: %s, Confidence: %.2f", cr.getClassName(), cr.getConfidence());
                    }

                    // Access detector results
                    List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
                    for (LLResultTypes.DetectorResult dr : detectorResults) {
                        telemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
                    }
                    }
                }
            } else {
                telemetry.addData("Limelight", "No data available");
            }
    }

    // Helper method to set motor powers on your specific robot
    /*static void setMotorPowersPF(MotorPowers powers) {
        ((DcMotorEx) leftFrontDrive).setPower(powers.frontLeft);
        ((DcMotorEx) leftBackDrive).setPower(powers.backLeft);
        ((DcMotorEx) rightFrontDrive).setPower(powers.frontRight);
        ((DcMotorEx) rightBackDrive).setPower(powers.backRight);
    }*/

    static void checkAlliance(HardwareMap hardwareMap) {
    /*   TouchSensor touchSensor1;  // Touch sensor Object
       touchSensor1 = hardwareMap.get(TouchSensor.class, "switch1");

         if (touchSensor1.isPressed()) {
             redAlliance = false;
         } else {
             redAlliance = true;
         }
         */
    }

     static double normalizeAngleD(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    // total velocity in inches/sec
    static double robotVelocity() {
        double xVel = ppVel.getX(DistanceUnit.INCH);
        double yVel = ppVel.getY(DistanceUnit.INCH);
        return Math.sqrt(xVel * xVel + yVel * yVel);
    }

    // total velocity in degrees/sec
    static double robotAngularVelocity() {
        return ppVel.getHeading(AngleUnit.DEGREES);
    }

    static boolean updatePosFromApril = false;
    // Update position from April tags if robot not moving
    static void updatePos() {
        if (!updatePosFromApril)
            return;
        double UPDATE_VELOCITY_THRESHOLD = 5;
        double UPDATE_ANGULAR_V_THRESHOLD = 5;
        /*if (isMt2Valid()) {
            if (robotVelocity()<UPDATE_VELOCITY_THRESHOLD && robotAngularVelocity()<UPDATE_ANGULAR_V_THRESHOLD) {
                /*odo.setPositionXYonly(new Pose2D(DistanceUnit.INCH,
                    getMt2X(),
                    getMt2Y(),
                    AngleUnit.DEGREES,
                    ppPos.getHeading(AngleUnit.DEGREES)));  // Angle should be ignored
                ppPosAbsolute++;
                //updatePinpoint();
            } else {
                ppPosThreshV++;
            }

        }*/
}

    static void dumpBucket() {
        liftMotor.setTargetPosition(2200);
        ((DcMotorEx) liftMotor).setVelocity(3000);
        liftPosition = 2200;
        while (liftMotor.getCurrentPosition() < 500) ; // Wait for motor to get to 500
        bucketServo.setPosition(BUCKET_UP);
        while (liftMotor.getCurrentPosition() < 2150) ; // Wait for motor to get to 2100
        sleep(300);
        bucketServo.setPosition(BUCKET_DOWN);
        sleep(100);
        liftMotor.setTargetPosition(40);
        liftPosition = 40;
    }

    static double error_x, error_y, error_h, scalefactor;

    static void updateMotors() {
        // Have current position in ppPos
        // Have current Velocity in ppVel
        // Have target position in targetPos

        double VEL_SCALE_FACTOR = 20;
        double ANG_SCALE_FACTOR = 20;

        error_x = targetPos.getX(DistanceUnit.INCH) - (ppPos.getX(DistanceUnit.INCH)+ppVel.getX(DistanceUnit.INCH)/VEL_SCALE_FACTOR);
        error_y = targetPos.getY(DistanceUnit.INCH) - (ppPos.getY(DistanceUnit.INCH)+ppVel.getY(DistanceUnit.INCH)/VEL_SCALE_FACTOR);
        error_h = normalizeAngleD(targetPos.getHeading(AngleUnit.DEGREES) - ((ppPos.getHeading(AngleUnit.DEGREES)-90)+ppVel.getHeading(AngleUnit.DEGREES)/ANG_SCALE_FACTOR));

        double current_heading_r = Math.toRadians(ppPos.getHeading(AngleUnit.DEGREES));
        double y = -error_y / 20;
        double x = error_x / 20;
        double rx = -error_h / 30;

            double rotX = y * Math.cos(-current_heading_r) - x * Math.sin(-current_heading_r);
            double rotY = y * Math.sin(-current_heading_r) + x * Math.cos(-current_heading_r);
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            ((DcMotorEx) leftFrontDrive).setVelocity(scalefactor * frontLeftPower);
            ((DcMotorEx) leftBackDrive).setVelocity(scalefactor * backLeftPower);
            ((DcMotorEx) rightFrontDrive).setVelocity(scalefactor * frontRightPower);
            ((DcMotorEx) rightBackDrive).setVelocity(scalefactor * backRightPower);
    }

    static void stopMotors() {

        ((DcMotorEx) leftFrontDrive).setVelocity(0);
        ((DcMotorEx) leftBackDrive).setVelocity(0);
        ((DcMotorEx) rightFrontDrive).setVelocity(0);
        ((DcMotorEx) rightBackDrive).setVelocity(0);
    }

    static double getRuntime() {
        return (System.currentTimeMillis()/1000.0);
    }

    static void sleep(int ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    static boolean debugMoves = false;

    // Move to x/y/h with acceptable error of x_err/y_err, h_err.  Timeout after timeout seconds even if position
    // not reached
    // Return true if move was successful, false if timed out
    // Speed is 0-100
    static Boolean moveToXYHsv(double x_target, double y_target, double h_target, double speed, double x_err, double y_err, double h_err, double v_max, double timeout) {

        targetPos = new Pose2D(DistanceUnit.INCH, x_target, y_target, AngleUnit.DEGREES, normalizeAngleD(h_target));
        scalefactor = 3000.0 * speed / 100.0;
        double startTime = getRuntime();

        while (true){
            telemetry.addData("Function", "moveToXYHs");

            //updatePinpoint();  // Gets current position
            checkLimelight();
            updatePos();

            updateMotors();
            if (Math.abs(error_x)<x_err && Math.abs(error_y)<y_err && Math.abs(error_h)<h_err && robotVelocity()<v_max){
                stopMotors();
                break;
            }

            if ((getRuntime()-startTime)>timeout) {
                stopMotors();
                return false;
            }

            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", targetPos.getX(DistanceUnit.INCH), targetPos.getY(DistanceUnit.INCH), targetPos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Target", data);

            data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", error_x, error_y, error_h);
            telemetry.addData("Error", data);
            telemetry.update();
        }

        if ((gamepad1!=null) && debugMoves) {
            while (!gamepad1.a) ;  // Wait for A to be pressed and then
            while (gamepad1.a) ;    // released before continuing
        }

        return true;
    }

    static Boolean moveRelXYHsv(double x_rel, double y_rel, double h_rel, double speed, double x_err, double y_err, double h_err, double v_max, double timeout) {
       return moveToXYHsv(ppPos.getX(DistanceUnit.INCH) + x_rel,
                    ppPos.getY(DistanceUnit.INCH) + y_rel,
                                        (ppPos.getHeading(AngleUnit.DEGREES)-90)+h_rel, speed, x_err, y_err, h_err, v_max, timeout);
    }


    // Takes in HSV array and returns b, r, y, or n for blue, red, yellow or none
    static char hsvToColor(float[] hsv) {
        double error_range = 20;
        double b_h = 215;
        double r_h = 25;
        double y_h = 75;
        char color;

        if (Math.abs(hsv[0] -b_h) < error_range){   color = 'b';     }
        else if (Math.abs(hsv[0] -r_h) < error_range){   color = 'r';     }
        else if (Math.abs(hsv[0] -y_h) < error_range){   color = 'y';     }
        else{ color = 'n';     }

        return color;
    }
}
