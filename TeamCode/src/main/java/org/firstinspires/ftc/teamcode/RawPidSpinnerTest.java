package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Raw PID Spinner Test", group = "Test")
public class RawPidSpinnerTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // --- 1. HARDWARE SETUP ---
        CRServo spinnerServo = hardwareMap.get(CRServo.class, "spinner");
        AnalogInput spinEncoder = hardwareMap.get(AnalogInput.class, "spinEncoder");

        // --- 2. PID & CONTROL VARIABLES ---

        // -- PID GAINS (The "brains" of the controller) --
        // kP (Proportional): The main driving force. Higher means more power for a given error.
        // If the servo moves AWAY from the target, make this number NEGATIVE (e.g., -0.015).
        double kP = 0.015;

        // kI (Integral): Corrects for small, steady errors if the servo stops just short.
        // Start with 0. Only add a tiny amount (e.g., 0.0005) if needed.
        double kI = 0.0;

        // kD (Derivative): Acts as a brake to prevent overshooting. Smooths the movement.
        double kD = 0.001;

        // -- Controller State Variables --
        double integralSum = 0;    // Stores the accumulated error over time.
        double lastError = 0;      // Stores the error from the previous loop cycle.
        double targetAngle = 0;    // The angle we want the servo to go to.

        // -- Timers --
        ElapsedTime pidTimer = new ElapsedTime();   // Measures time between PID loops.
        ElapsedTime stepTimer = new ElapsedTime();  // Measures time for automatic stepping.

        telemetry.addLine("Setup Complete. Ready to Start.");
        telemetry.addLine("This test will automatically step the servo in 60-degree increments.");
        telemetry.update();

        waitForStart();

        // --- 3. MAIN CONTROL LOOP ---
        pidTimer.reset();
        stepTimer.reset();

        while (opModeIsActive()) {

            // --- Automatic Stepping Logic ---
            // Every 4 seconds, update the target angle.
            if (stepTimer.seconds() >= 4.0) {
                targetAngle += 60;
                stepTimer.reset();
                integralSum = 0; // Reset integral sum when the target changes.
            }

            // --- PID CALCULATION (The Core Logic) ---

            // Get the current angle from the analog encoder.
            // Assumes 0-3.3V maps to 0-360 degrees. Adjust if your encoder is different.
            double currentAngle = (spinEncoder.getVoltage() / 3.3) * 360.0;

            // Calculate the error (how far are we from the target?).
            double error = targetAngle - currentAngle;

            // Handle "angle wrapping" to ensure the servo takes the shortest path.
            if (error > 180) {
                error -= 360;
            } else if (error < -180) {
                error += 360;
            }

            // Get the time elapsed since the last loop (dt - delta time).
            double dt = pidTimer.seconds();
            pidTimer.reset();
            if (dt == 0) dt = 0.001; // Avoid division by zero on the first loop.

            // Calculate the Integral (sum of past errors).
            integralSum += error * dt;

            // Calculate the Derivative (rate of change of error).
            double derivative = (error - lastError) / dt;
            lastError = error;

            // Calculate the total power using the PID formula.
            double power = (kP * error) + (kI * integralSum) + (kD * derivative);

            // Limit the power to a reasonable range (e.g., -0.7 to 0.7).
            power = Range.clip(power, -0.7, 0.7);

            // --- SET SERVO POWER ---
            spinnerServo.setPower(power);

            // --- TELEMETRY ---
            telemetry.addData("Target Angle", "%.1f", targetAngle);
            telemetry.addData("Current Angle", "%.1f", currentAngle);
            telemetry.addData("Error", "%.1f", error);
            telemetry.addData("Calculated Power", "%.2f", power);
            telemetry.addData("Time until next step", "%.1f", 4.0 - stepTimer.seconds());
            telemetry.update();
        }
    }
}
