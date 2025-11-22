package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.HeadingInterpolator;
//trust
public class PathFactory {
    public static PathChain createCirclePath(Follower follower, double radius) {
        return follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(0, 0), new Pose(radius, 0), new Pose(radius, radius)))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(0, radius))
                .addPath(new BezierCurve(new Pose(radius, radius), new Pose(radius, 2 * radius), new Pose(0, 2 * radius)))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(0, radius))
                .addPath(new BezierCurve(new Pose(0, 2 * radius), new Pose(-radius, 2 * radius), new Pose(-radius, radius)))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(0, radius))
                .addPath(new BezierCurve(new Pose(-radius, radius), new Pose(-radius, 0), new Pose(0, 0)))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(0, radius))
                .build();
    }
}
