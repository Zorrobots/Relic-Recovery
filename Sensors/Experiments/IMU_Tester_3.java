package org.firstinspires.ftc.teamcode.RelicRecovery.Sensors.Experiments;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RelicRecovery.Autonomous.Tester_Autonomous;

import java.util.Locale;

/**
 * Created by Jorge on 19/01/2018.
 */
//@Autonomous(name = "IMU_Tester_3", group = "Sensor")
public class IMU_Tester_3 extends LinearOpMode {
    BNO055IMU imu;
    Orientation angles;
    double Angulos;
    double YawAngle;
    public DcMotor GlyphGrabberR = null;
    public DcMotor GlyphGrabberL = null;


    @Override
    public void runOpMode() throws InterruptedException {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        GlyphGrabberL = hardwareMap.dcMotor.get("GlyphGrabberL");
        GlyphGrabberR = hardwareMap.dcMotor.get("GlyphGrabberR");
        GlyphGrabberL.setDirection(DcMotor.Direction.REVERSE);
        imu.initialize(parameters);

        composeTelemetry();

        waitForStart();
        while (opModeIsActive()) {
            YawAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
            //YawAngle =  angles.firstAngle;
            telemetry.addData("YawAngle: ", YawAngle);
            telemetry.update();
          if (YawAngle > 0) {
                MoveMotors(1, 1);
            } else if (YawAngle < 0) {
                MoveMotors(-1, -1);
            } else {
                MoveMotors(0, 0);
            }

        }
    }
    void MoveMotors (double powerL, double powerR){
        GlyphGrabberL.setPower(powerL);
        GlyphGrabberR.setPower(powerR);
    }
    void composeTelemetry() {

        telemetry.addLine().addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                }).addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });
        telemetry.addLine().addData("heading", new Func<String>() {
            @Override public String value() {return formatAngle(angles.angleUnit, angles.firstAngle);}
        });
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
