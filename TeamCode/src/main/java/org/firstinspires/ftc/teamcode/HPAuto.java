package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder.*;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class HPAuto {
    private Follower follower;
    PathChain placeFirst = follower.pathBuilder()
        .addPath(
            // Line 1
            new BezierLine(
                new Point(9.000, 48.000, Point.CARTESIAN),
                new Point(36.692, 66.692, Point.CARTESIAN)
            )
        )
        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
    .build();
    
    PathChain slideBlocks = follower.pathBuilder()
        .addPath(
            // Line 2
            new BezierCurve(
                new Point(36.692, 66.692, Point.CARTESIAN),
                new Point(22.154, 20.077, Point.CARTESIAN),
                new Point(70.385, 46.154, Point.CARTESIAN),
                new Point(67.846, 23.769, Point.CARTESIAN)
            )
        )
        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
        .addPath(
            // Line 3
            new BezierLine(
                new Point(67.846, 23.769, Point.CARTESIAN),
                new Point(17.769, 23.769, Point.CARTESIAN)
            )
        )
        .setConstantHeadingInterpolation(Math.toRadians(180))
        .addPath(
            // Line 4
            new BezierCurve(
                new Point(17.769, 23.769, Point.CARTESIAN),
                new Point(70.846, 25.385, Point.CARTESIAN),
                new Point(67.615, 13.385, Point.CARTESIAN)
            )
        )
        .setConstantHeadingInterpolation(Math.toRadians(180))
        .addPath(
            // Line 5
            new BezierLine(
                new Point(67.615, 13.385, Point.CARTESIAN),
                new Point(17.769, 13.385, Point.CARTESIAN)
            )
        )
        .setConstantHeadingInterpolation(Math.toRadians(180))
        .addPath(
            // Line 6
            new BezierCurve(
                new Point(17.769, 13.385, Point.CARTESIAN),
                new Point(71.769, 15.462, Point.CARTESIAN),
                new Point(67.846, 8.077, Point.CARTESIAN)
            )
        )
        .setConstantHeadingInterpolation(Math.toRadians(180))
        .addPath(
            // Line 7
            new BezierLine(
                new Point(67.846, 8.077, Point.CARTESIAN),
                new Point(8.900, 8.308, Point.CARTESIAN)
            )
        )
        .setConstantHeadingInterpolation(Math.toRadians(180))
    .build();
}
