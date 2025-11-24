package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name="AprilTagSimple", group="Vision")
public class AprilTagSimple extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    @Override
    public void runOpMode() {

        // 1) Make an AprilTag processor with default settings.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // 2) Create the VisionPortal using your USB webcam named "Webcam 1".
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                aprilTag
        );

        telemetry.addLine("AprilTag init done – press PLAY");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            
            List<AprilTagDetection> detections = aprilTag.getDetections();

            telemetry.addData("# tags", detections.size());

            if (!detections.isEmpty()) {
                AprilTagDetection tag = detections.get(0);

                telemetry.addData("ID", tag.id);
                telemetry.addData("x (in)", "%.1f", tag.ftcPose.x);
                telemetry.addData("y (in)", "%.1f", tag.ftcPose.y);
                telemetry.addData("heading (deg)", "%.1f", tag.ftcPose.yaw);
            } else {
                telemetry.addLine("No tags");
            }

            telemetry.update();
            sleep(20);
        }

        // Clean up camera when you're done.
        visionPortal.close();
    }
}
