package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Timer;

import Wheelie.Path;
import Wheelie.PathFollower;
import Wheelie.Pose2D;

@Autonomous
public class TestPursuit extends LinearOpMode {
    private MecDrivebase drive;
    private PathFollower follower;

    private Pose2D[] points = new Pose2D[] { //TODO changes these
            new Pose2D(24, 0,0),
            new Pose2D(24, 24, Math.toRadians(90)),
    };

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2D start = new Pose2D(0,0,0);
        Path path = new Path(start, points);

        follower = new PathFollower(start, 12, path);

        drive = new MecDrivebase(hardwareMap, start);
        double time = 0;

        waitForStart();

        while (!concludePath(path.getPt(-1))){
            drive.update();

            Pose2D move = follower.followPath(drive.getPose());
            if (!follower.isShrinkingLook()){
                drive.moveTo(move);
            } else {
                telemetry.addLine("Using PID");
                if(time == 0)
                    time = System.currentTimeMillis();
                drive.moveToPID(move, path.getPt(-1), time);
            }

            telemetry.addLine("Waypoint #" + (follower.getWayPoint() + 1));
            telemetry.update();
        }
    }

    private boolean concludePath(Pose2D target){
        return distance(target, drive.getPose()) <= 2 && follower.isShrinkingLook();
    }

    private double distance(Pose2D a, Pose2D b){
        return Math.hypot(a.x-b.x, a.y-b.y);
    }
}
