package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import Wheelie.Pose2D;

public class Localization {
    private DcMotorEx hori, vert;
    private BNO055IMU imu;
    Orientation angles;

    private int prevH, prevV;

    //In millimeters
    public final static double WHEEL_DIAMETER = 48.0,
                                WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    public final static int TICKS_PER_REV = 2000;
    public final static double MM_TO_INCH = 1.0/25.4;
    public final static double MM_PER_TICK = WHEEL_CIRCUMFERENCE / TICKS_PER_REV;
    // In inches
    public final static double H_DISTANCE_FROM_MID = 6; //TODO Get real values
    public final static double V_DISTANCE_FROM_MID = 7;

    public Pose2D currentPosition;

    public Localization(HardwareMap hw, Pose2D start){
        hori = hw.get(DcMotorEx.class, "suspend");
        vert = hw.get(DcMotorEx.class, "Intake");
        hori.setDirection(DcMotorSimple.Direction.REVERSE);
        vert.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        imu = hw.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        currentPosition = new Pose2D(start.x, start.y, start.h);
    }

    private void calculateChanges() {
        int currentH = hori.getCurrentPosition();
        int currentV = vert.getCurrentPosition();
        int dy = currentH - prevH;
        int dx = currentV - prevV;
        double heading = getAngle();

        // Convert ticks to millimeters
        double dH = dy * MM_PER_TICK;
        double dV = dx * MM_PER_TICK;

        // Calculate the change in orientation (heading)
        double deltaHeading = heading - currentPosition.h;

        // Calculate the translation components
        double forward = dV - V_DISTANCE_FROM_MID * deltaHeading;
        double strafe = dH + H_DISTANCE_FROM_MID * deltaHeading;

        // Apply the rotation to the translation to convert to global coordinates
        double globalForward = forward * Math.cos(heading) - strafe * Math.sin(heading);
        double globalStrafe = forward * Math.sin(heading) + strafe * Math.cos(heading);

        // Update the current position
        currentPosition.x += globalForward * MM_TO_INCH;
        currentPosition.y += globalStrafe * MM_TO_INCH;
        currentPosition.h = heading;

        // Update previous encoder values
        prevH = currentH;
        prevV = currentV;
    }


    public double getAngle(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return (angles.firstAngle + 2 * Math.PI) % (2 * Math.PI);
    }

    public void update(){
        calculateChanges();
    }
}