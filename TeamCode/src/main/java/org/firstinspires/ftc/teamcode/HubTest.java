package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

@TeleOp(name = "HubTest")
public class HubTest extends LinearOpMode {
    public void runOpMode() throws InterruptedException {

        // All ports default to NONE, buses default to empty
        SRSHub.Config config = new SRSHub.Config();


        config.addI2CDevice(
            1,
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

                SRSHub.VL53L5CX lidar = hub.getI2CDevice(
                    1,
                    SRSHub.VL53L5CX.class
                );

                if (!lidar.disconnected) {
                    telemetry.addData(
                        "Lidar Device",
                        lidar.toString()
                    );

                    telemetry.addData(
                            "Lidar Resolution",
                            lidar.distances.length == 16 ? "4x4" : "8x8"
                    );

                    // Show 4 rows of LIDAR distances, 4 per row
                    for (int row = 0; row < 4; row++) {
                        StringBuilder rowData = new StringBuilder();
                        for (int col = 0; col < 4; col++) {
                            int index = row * 4 + col;
                            rowData.append(String.format("%4d ", lidar.distances[index]));
                        }
                        telemetry.addData("Row " + row, rowData.toString());
                    }

                }
            }

            telemetry.update();
        }
    }
}