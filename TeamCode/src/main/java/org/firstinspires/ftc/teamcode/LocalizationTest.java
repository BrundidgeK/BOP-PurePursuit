package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Wheelie.PathFollower;
import Wheelie.Pose2D;

public class LocalizationTest extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        MecDrivebase drive = new MecDrivebase(hardwareMap, new Pose2D(0,0,0));

        waitForStart();

        while (opModeIsActive()){
            drive.update();
            telemetry.addLine(drive.getPoseString());
        }
    }
}
