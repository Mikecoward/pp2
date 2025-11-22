package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawOnlyCurrent;

import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.PathFactory;

@Autonomous(name = "AS-autobot1", group = "Concept")
public class AutoBot extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        follower.setStartingPose(new com.pedropathing.geometry.Pose(0,0,0));

        PathChain circle = PathFactory.createCirclePath(follower, 10); // radius 10

        telemetry.addLine("Ready to run circular path");
        telemetry.update();
        waitForStart();

        follower.followPath(circle);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            drawOnlyCurrent();
        }

        follower.breakFollowing();
    }
}
