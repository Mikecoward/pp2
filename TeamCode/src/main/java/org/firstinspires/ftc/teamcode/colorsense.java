package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="colorsense", group="color")

public class colorsense extends LinearOpMode {

    // Raw averaged calibration data
    private static int noneR, noneG, noneB;
    private static int greenR, greenG, greenB;
    private static int purpleR, purpleG, purpleB;

    // Thresholds generated automatically
    private static int thresholdG_noneToColors;
    private static int thresholdB_greenToPurple;

    // Values read each frame
    public static int redValue, greenValue, blueValue;


    // ============================================================
    // PART 1 — CALIBRATION INIT (CALL THIS ON INIT)
    // ============================================================
    public static void calibrate(LinearOpMode opMode) {

        final int SAMPLE_COUNT = 50;

        // ========================================================
        // STEP 1 — CALIBRATE NONE
        // ========================================================
        opMode.telemetry.addLine("Place sensor on NONE and press A");
        opMode.telemetry.update();
        while (!opMode.gamepad1.a && !opMode.isStopRequested()) {}

        int sumR = 0, sumG = 0, sumB = 0;
        for (int i = 0; i < SAMPLE_COUNT; i++) {
            redValue   = Common.colorsense.red();
            greenValue = Common.colorsense.green();
            blueValue  = Common.colorsense.blue();

            sumR += redValue;
            sumG += greenValue;
            sumB += blueValue;
            opMode.sleep(10);
        }
        noneR = sumR / SAMPLE_COUNT;
        noneG = sumG / SAMPLE_COUNT;
        noneB = sumB / SAMPLE_COUNT;
        opMode.telemetry.addLine(String.valueOf(noneR));
        opMode.telemetry.addLine(String.valueOf(noneG));
        opMode.telemetry.addLine(String.valueOf(noneB));
        opMode.telemetry.update();
        opMode.sleep(2000);

        // ========================================================
        // STEP 2 — CALIBRATE GREEN
        // ========================================================
        opMode.telemetry.addLine("Place sensor on GREEN and press B");
        opMode.telemetry.update();
        while (!opMode.gamepad1.b && !opMode.isStopRequested()) {}

        sumR = sumG = sumB = 0;
        for (int i = 0; i < SAMPLE_COUNT; i++) {
            redValue   = Common.colorsense.red();
            greenValue = Common.colorsense.green();
            blueValue  = Common.colorsense.blue();

            sumR += redValue;
            sumG += greenValue;
            sumB += blueValue;
            opMode.sleep(10);
        }
        greenR = sumR / SAMPLE_COUNT;
        greenG = sumG / SAMPLE_COUNT;
        greenB = sumB / SAMPLE_COUNT;
        opMode.telemetry.addLine(String.valueOf(greenR));
        opMode.telemetry.addLine(String.valueOf(greenG));
        opMode.telemetry.addLine(String.valueOf(greenB));
        opMode.telemetry.update();
        opMode.sleep(2000);

        // ========================================================
        // STEP 3 — CALIBRATE PURPLE
        // ========================================================
        opMode.telemetry.addLine("Place sensor on PURPLE and press X");
        opMode.telemetry.update();
        while (!opMode.gamepad1.x && !opMode.isStopRequested()) {}

        sumR = sumG = sumB = 0;
        for (int i = 0; i < SAMPLE_COUNT; i++) {
            redValue   = Common.colorsense.red();
            greenValue = Common.colorsense.green();
            blueValue  = Common.colorsense.blue();

            sumR += redValue;
            sumG += greenValue;
            sumB += blueValue;
            opMode.sleep(10);
        }
        purpleR = sumR / SAMPLE_COUNT;
        purpleG = sumG / SAMPLE_COUNT;
        purpleB = sumB / SAMPLE_COUNT;

        opMode.telemetry.addLine(String.valueOf(purpleR));
        opMode.telemetry.addLine(String.valueOf(purpleG));
        opMode.telemetry.addLine(String.valueOf(purpleB));
        opMode.telemetry.update();
        opMode.sleep(2000);


        // ========================================================
        // GENERATE THRESHOLDS
        // ========================================================

        thresholdG_noneToColors = (noneG + greenG) / 2;
        thresholdB_greenToPurple = (purpleB + greenB) / 2;


        // ========================================================
        // OUTPUT CALIBRATION
        // ========================================================
        opMode.telemetry.addLine("=== CALIBRATION COMPLETE ===");

        opMode.telemetry.addData("NONE",   "%d %d %d", noneR, noneG, noneB);
        opMode.telemetry.addData("GREEN",  "%d %d %d", greenR, greenG, greenB);
        opMode.telemetry.addData("PURPLE", "%d %d %d", purpleR, purpleG, purpleB);

        opMode.telemetry.addLine("=== Thresholds ===");
        opMode.telemetry.addData("G none→color", thresholdG_noneToColors);
        opMode.telemetry.addData("B green↔purple", thresholdB_greenToPurple);

        opMode.telemetry.update();
        opMode.sleep(500);
    }


    // ============================================================
    // PART 2 — DECISION TREE CLASSIFIER
    // CALL THIS DURING LOOP()
    // ============================================================
    public static String classify(LinearOpMode opMode) {

        // Read fresh sensor values
        redValue   = Common.colorsense.red();
        greenValue = Common.colorsense.green();
        blueValue  = Common.colorsense.blue();

        // OUTER IF → ELSE (AS YOU REQUIRED)
        if (greenValue < thresholdG_noneToColors) {

            // NONE branch
            if (redValue < noneR + 50) {
                if (blueValue < noneB + 50) {
                    opMode.telemetry.addData("color", "none");
                    return "none";
                } else {
                    opMode.telemetry.addData("color", "none");
                    return "none";
                }
            } else {
                opMode.telemetry.addData("color", "none");
                return "none";
            }

        } else {

            // GREEN OR PURPLE branch
            if (blueValue < thresholdB_greenToPurple) {

                // PURPLE
                if (greenValue > purpleG - 100) {
                    opMode.telemetry.addData("color", "purple");
                    return "purple";
                } else {
                    opMode.telemetry.addData("color", "purple");
                    return "purple";
                }

            } else {

                // GREEN
                if (greenValue > greenG - 200) {
                    opMode.telemetry.addData("color", "green");
                    return "green";
                } else {
                    opMode.telemetry.addData("color", "green");
                    return "green";
                }
            }
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
