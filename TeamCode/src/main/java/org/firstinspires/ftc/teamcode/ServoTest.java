package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Test", group = "Testing")
public class ServoTest extends LinearOpMode {
    private Servo catstrength;
    private double servoPosition = 0.75;

    private static final double MIN_POSITION = 0.2;
    private static final double MAX_POSITION = 0.75;
    private static final double INCREMENT = 0.01;

    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize servo
        catstrength = hardwareMap.get(Servo.class, "catstrength");
        catstrength.setPosition(servoPosition);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Servo Position", "%.2f", servoPosition);
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("  DPad Up: Increase position");
        telemetry.addLine("  DPad Down: Decrease position");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // DPad Up - Increase position
            if (gamepad1.dpad_up && !dpadUpPressed) {
                servoPosition += INCREMENT;
                if (servoPosition > MAX_POSITION) {
                    servoPosition = MAX_POSITION;
                }
                catstrength.setPosition(servoPosition);
                dpadUpPressed = true;
            } else if (!gamepad1.dpad_up) {
                dpadUpPressed = false;
            }

            // DPad Down - Decrease position
            if (gamepad1.dpad_down && !dpadDownPressed) {
                servoPosition -= INCREMENT;
                if (servoPosition < MIN_POSITION) {
                    servoPosition = MIN_POSITION;
                }
                catstrength.setPosition(servoPosition);
                dpadDownPressed = true;
            } else if (!gamepad1.dpad_down) {
                dpadDownPressed = false;
            }

            // Display current position
            telemetry.addData("Servo Position", "%.2f", servoPosition);
            telemetry.addData("Range", "%.2f - %.2f", MIN_POSITION, MAX_POSITION);
            telemetry.addLine();
            telemetry.addLine("Controls:");
            telemetry.addLine("  DPad Up: Increase (+0.01)");
            telemetry.addLine("  DPad Down: Decrease (-0.01)");
            telemetry.update();

            sleep(50); // Small delay to prevent excessive updates
        }
    }
}
