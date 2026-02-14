package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

@TeleOp(name = "HubTest")
public class HubTest extends LinearOpMode {
    public void runOpMode() throws InterruptedException {

        // All ports default to NONE, buses default to empty
        SRSHub.Config config = new SRSHub.Config();

        // First VL53L5CX sensor on bus 1
        config.addI2CDevice(
            1,
            new SRSHub.VL53L5CX(SRSHub.VL53L5CX.Resolution.GRID_4x4)
        );

        // Second VL53L5CX sensor on bus 2
        config.addI2CDevice(
            2,
            new SRSHub.VL53L5CX(SRSHub.VL53L5CX.Resolution.GRID_4x4)
        );

        /*config.addI2CDevice(
            1,
            new SRSHub.GoBildaPinpoint(
                -50,
                -75,
                19.89f,
                SRSHub.GoBildaPinpoint.EncoderDirection.FORWARD,
                SRSHub.GoBildaPinpoint.EncoderDirection.FORWARD
            )
        );
        */

        RobotLog.clearGlobalWarningMsg();

        SRSHub hub = hardwareMap.get(
            SRSHub.class,
            "srshub"
        );

        hub.init(config);

        while (!hub.ready());

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            hub.update();

            if (hub.disconnected()) {
                telemetry.addLine("srshub disconnected");
            } else {

                // Read first sensor on bus 1
                SRSHub.VL53L5CX lidar1 = hub.getI2CDevice(
                    1,
                    SRSHub.VL53L5CX.class
                );

                // Read second sensor on bus 2
                SRSHub.VL53L5CX lidar2 = hub.getI2CDevice(
                    2,
                    SRSHub.VL53L5CX.class
                );

                // Display first sensor
                telemetry.addLine("=== SENSOR 1 (Bus 1) ===");
                if (!lidar1.disconnected) {
                    telemetry.addData(
                            "Resolution",
                            lidar1.distances.length == 16 ? "4x4" : "8x8"
                    );

                    // Show 4 rows of LIDAR distances, 4 per row
                    for (int row = 0; row < 4; row++) {
                        StringBuilder rowData = new StringBuilder();
                        for (int col = 0; col < 4; col++) {
                            int index = row * 4 + col;
                            rowData.append(String.format("%4d ", lidar1.distances[index]));
                        }
                        telemetry.addData("S1 Row " + row, rowData.toString());
                    }
                } else {
                    telemetry.addLine("Sensor 1 DISCONNECTED");
                }

                telemetry.addLine();

                // Display second sensor
                telemetry.addLine("=== SENSOR 2 (Bus 2) ===");
                if (!lidar2.disconnected) {
                    telemetry.addData(
                            "Resolution",
                            lidar2.distances.length == 16 ? "4x4" : "8x8"
                    );

                    // Show 4 rows of LIDAR distances, 4 per row
                    for (int row = 0; row < 4; row++) {
                        StringBuilder rowData = new StringBuilder();
                        for (int col = 0; col < 4; col++) {
                            int index = row * 4 + col;
                            rowData.append(String.format("%4d ", lidar2.distances[index]));
                        }
                        telemetry.addData("S2 Row " + row, rowData.toString());
                    }
                } else {
                    telemetry.addLine("Sensor 2 DISCONNECTED");
                }
            }

            telemetry.update();
        }
    }
}