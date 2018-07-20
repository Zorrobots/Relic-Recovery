package org.firstinspires.ftc.teamcode.RelicRecovery.Sensors.Practicals;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="IMU_Gyro_Turn", group="Exercises")
public class IMU_Gyro_Turn extends LinearOpMode
{
    public DcMotor L_M_F = null;
    public DcMotor L_M_B = null;
    public DcMotor R_M_F = null;
    public DcMotor R_M_B = null;
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double globalAngle, power = .30, correction;

    @Override
    public void runOpMode() throws InterruptedException
    {
        L_M_F = hardwareMap.dcMotor.get("L_M_F");
        L_M_B = hardwareMap.dcMotor.get("L_M_B");
        R_M_F = hardwareMap.dcMotor.get("R_M_F");
        R_M_B = hardwareMap.dcMotor.get("R_M_B");

        L_M_F.setDirection(DcMotor.Direction.FORWARD);
        L_M_B.setDirection(DcMotor.Direction.FORWARD);
        R_M_B.setDirection(DcMotor.Direction.REVERSE);
        R_M_F.setDirection(DcMotor.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        while (!isStopRequested() && !imu.isGyroCalibrated()){
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        waitForStart();
        ImuTurnRed(-90);
        sleep(1000);
        ImuTurnRed(0);
        sleep(1000);
        ImuTurnRed(90);
        sleep(1000);
        }
        private void ImuTurnRed(int targetAngle){
    getAngle();
    telemetry.addData("Z first", lastAngles.firstAngle);
    telemetry.update();
    sleep(3000);
    while (lastAngles.firstAngle > targetAngle){
        R_M_F.setPower(.3);
        R_M_B.setPower(.3);
        L_M_F.setPower(-.3);
        L_M_B.setPower(-.3);
        getAngle();
        telemetry.addData("Z first", lastAngles.firstAngle);
        telemetry.update();
    }
    R_M_F.setPower(0);
    R_M_B.setPower(0);
    L_M_F.setPower(0);
    L_M_B.setPower(0);
    telemetry.addLine("Finished");
    telemetry.addData("Final Position: ",lastAngles.firstAngle);
    telemetry.update();
    sleep(3000);
}
        private double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
}