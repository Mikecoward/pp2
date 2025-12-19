/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import java.util.List;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

import java.util.List;


/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list

 0 left Front
 1 Right Front
 2 Right Back
 3 Left Back

 Expansion
 0 r cat
 1 l cat
 2 intake
 3 lifter
 */

// Based on the sample: Basic: Omni Linear OpMode
@TeleOp(name = "Catapult Test", group = "Teleop")

public class CatapultTest extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime catatime = new ElapsedTime();
    //private Limelight3A limelight;

    // Declare drive motors
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;

    // Declare end-effector members
    private DcMotorEx intake = null;
    private DcMotorEx catapult1 = null;
    private DcMotorEx catapult2 = null;

    // motor power 1 = 100% and 0.5 = 50%
    // negative values = reverse ex: -0.5 = reverse 50%
    private double INTAKE_IN_POWER = -1;
    private double INTAKE_OUT_POWER = 0.9;
    private double INTAKE_OFF_POWER = 0.0;
    private double intakePower = INTAKE_OFF_POWER;
    private static final double DRIVE_TICKS_PER_SEC_MAX = 2800.0;

    private double CATAPULT_UP_POWER = -1;
    private double CATAPULT_DOWN_POWER = 1;
    private double CATAPULT_HOLD_POWER = 0.0;
    private double CATAPULT_HOLD_DOWN_POWER = 0.2;

    private boolean catapultbuttonpresssed = false;

    private enum CatapultModes {UP, DOWN, HOLD}
    private CatapultModes pivotMode;


    static GoBildaPinpointDriver odo;
    static Pose2D ppPos, ppVel, targetPos;
    static int ppPosAbsolute = 0;  // 1Number of updates.  0 is relative, 1+ is absolute
    static int ppPosThreshV = 0;    // Velocity blocked
    static int ppPosThreshS = 0;    // Fiducial size blocked
    boolean catapultUp = true;

    boolean headingfield = false;
    private Limelight3A limelight;
    /*
     * Code to run ONCE when the driver hits INIT (same as previous year's init())
     */
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        /*limelight = hardwareMap.get(Limelight3A.class, "limelight");*/

        //Limelight3A limelight = new Limelight3A(172.29.0.23);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();

        telemetry.setMsTransmissionInterval(11);

        //limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.
         */
        //limelight.start();


        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        odo.resetPosAndIMU();
        configurePinpoint(hardwareMap, true);


        // Set pose to X=0 mm, Y=0 mm, Heading=90 degrees
        odo.setPosition(new Pose2D(
                DistanceUnit.MM,
                0.0,
                0.0,
                AngleUnit.DEGREES,
                90.0
        ));


        IMU imu;
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        leftFrontDrive = (DcMotorEx) hardwareMap.get(DcMotor.class, "lf");
        leftBackDrive = (DcMotorEx) hardwareMap.get(DcMotor.class, "lb");
        rightFrontDrive =(DcMotorEx)  hardwareMap.get(DcMotor.class, "rf");
        rightBackDrive = (DcMotorEx) hardwareMap.get(DcMotor.class, "rb");

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake = (DcMotorEx)hardwareMap.get(DcMotor.class, "intake");
        catapult1 = (DcMotorEx)hardwareMap.get(DcMotor.class, "rcat");
        catapult2 = (DcMotorEx)hardwareMap.get(DcMotor.class, "lcat");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed
        // to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.

        // set direction of wheel motors
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // set direction of subsystem motors
        intake.setDirection(DcMotor.Direction.FORWARD); // Forward should INTAKE.
        catapult1.setDirection(DcMotor.Direction.REVERSE); // Backwards should pivot DOWN, or in the stowed position.
        catapult2.setDirection(DcMotor.Direction.FORWARD);

        // set initial subsystem behavior
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        catapult1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        catapult2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        catapult1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        catapult2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        catapult1.setTargetPosition(0);
        catapult2.setTargetPosition(0);
        catapult1.setPower(1.0);
        catapult2.setPower(1.0);

        catapult1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        catapult2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        catatime.reset();
        double catpos = 0;

        while (opModeIsActive()) {
            catapult1.setTargetPosition((int)catpos);
            catapult2.setTargetPosition((int)catpos);
            catapult1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            catapult2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            if (gamepad1.dpad_up) {
                catpos += 0.1;
            }
            if (gamepad1.dpad_down) {
                catpos -= 0.1;
            }

            if (gamepad1.dpad_right) {
                catpos = 0;
            }

            if (gamepad1.dpad_left) {
                catpos = 90;
            }


            // UPDATE TELEMETRY
            // Show the elapsed game time, wheel power, and other systems power
            telemetry.addData("Cat pos: ", catpos);
            telemetry.addData("Catapult1 Current/Target/power/mA", "%d, %d, %4.2f, %4.2f",
                    catapult1.getCurrentPosition(), catapult1.getTargetPosition(), catapult1.getPower(), catapult1.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("Catapult2 Current/Target/power/mA", "%d, %d, %4.2f, %4.2f",
                    catapult2.getCurrentPosition(), catapult2.getTargetPosition(), catapult2.getPower(), catapult2.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("Tol", catapult1.getTargetPositionTolerance());
            telemetry.update();
        }
    }


    static void configurePinpoint(HardwareMap hardwareMap, boolean recalibrateIMU) {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        //odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");


        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        odo.setOffsets(-98.0, 150.0, DistanceUnit.MM);

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per mm of your odometry pod.
         */
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);


        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */

        if (recalibrateIMU) {
            odo.recalibrateIMU();
            sleep(500);
            odo.resetPosAndIMU();
            sleep(500);
        }

    }

    static void sleep(int ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    void updatePinpoint() {
        odo.update();

        ppPos = odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f, A: %d, V: %d, F: %d}", ppPos.getX(DistanceUnit.INCH), ppPos.getY(DistanceUnit.INCH), normalizeAngleD(ppPos.getHeading(AngleUnit.DEGREES)-90), ppPosAbsolute, ppPosThreshV, ppPosThreshS);
        telemetry.addData("Position", data);


        //gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.

/*            ppVel = odo.getVelocity();
            String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", ppVel.getX(DistanceUnit.MM), ppVel.getY(DistanceUnit.MM), ppVel.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Velocity", velocity);
*/

            /*Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll primarily see
            READY: the device is working as normal
            CALIBRATING: the device is calibrating and outputs are put on hold
            NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
            FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
            FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
            FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in
            */

        telemetry.addData("Status", odo.getDeviceStatus());
    }

    static double normalizeAngleD(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

}
