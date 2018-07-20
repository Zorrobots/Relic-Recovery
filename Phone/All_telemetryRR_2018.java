package org.firstinspires.ftc.teamcode.RelicRecovery.Phone;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


/**
 * Created by Jorge on 23/11/2017.
 */
@TeleOp(name="All_telemetryRR_2018")
public class All_telemetryRR_2018 extends LinearOpMode {

    protected String Multi;
    protected String JewelColor;
    protected String StoneColor;
    protected String blue = "blue";
    protected String red = "red";
    protected double globalAngle;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    //SENSORS
    protected ColorSensor LineSensor;
    protected ColorSensor JewelSensor;
    protected ModernRoboticsI2cGyro GyroSensor;
    protected ModernRoboticsI2cGyro GyroS;
    protected ModernRoboticsI2cRangeSensor RangeSensor;
    protected VuforiaLocalizer vuforia;

    protected DcMotor L_M_F = null;
    protected DcMotor L_M_B = null;
    protected DcMotor R_M_F = null;
    protected DcMotor R_M_B = null;

    @Override
    public void runOpMode() throws InterruptedException {
        //HARDWARE MAPPING
        JewelSensor = hardwareMap.colorSensor.get("JewelSensor");
        LineSensor = hardwareMap.colorSensor.get("LineSensor");
        GyroSensor = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("GyroSensor");
        GyroS = (ModernRoboticsI2cGyro) GyroSensor;
        RangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "RangeSensor");
        LineSensor.enableLed(true);
        JewelSensor.enableLed(true);
        L_M_F = hardwareMap.dcMotor.get("L_M_F");
        L_M_B = hardwareMap.dcMotor.get("L_M_B");
        R_M_F = hardwareMap.dcMotor.get("R_M_F");
        R_M_B = hardwareMap.dcMotor.get("R_M_B");
        R_M_F.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R_M_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L_M_F.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L_M_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();

        BNO055IMU.Parameters parametersImu = new BNO055IMU.Parameters();
        parametersImu.mode                = BNO055IMU.SensorMode.IMU;
        parametersImu.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parametersImu.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersImu.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parametersImu);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        while (!isStopRequested() && !imu.isGyroCalibrated()){
            sleep(50);
            idle();
        }
        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AYy3kgb/////AAAAGfahknPNfEOHpk9drxT3x5s+h85enQDuwX5Y/R9chthrPe1AQ1A+iYyS9PoUpVOVcu4TM/lzJa/PqlyaHKJWh+fI63xLIftsjqQ15b+MoQNZrgG4sw0swD9/yYSfSn3AU6PuQ6OozHZf4zrEOiL2AL/1OMMxbd9KddgiIIX5X/rnx7VFMFiNR8vq+otCHameCqnzdRcCkp1rqo+bewMyMYjTeYIyl29wn0oElYjg1PdBoYgDiUIjQu4sVECgCH7c6+pmEYe37ypfeMCxoGmG60L8bUmq5RrzZ1mxdJkugZ4hRbG/UIm1aHApSHE+ljsAexK3crM78qRfdVK6B9PTsnEEq9C40FuYu/ZqcCglO5VZ\n";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        waitForStart();
        while  (opModeIsActive()){
            relicTrackables.activate();
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            telemetry.addData(">  VuMark", "%s visible", vuMark);
            CheckJewelColor();
            CheckStoneColor();
            CheckRangeDistanceCM();
            CheckHeading();
            CheckAngle();
            FourEncoders();
            telemetry.update();
        }
    }
    //FUNCTIONS
    protected void CheckJewelColor(){
        float hsvValues[] = {0, 0, 0};

        Color.RGBToHSV(JewelSensor.red() * 8, JewelSensor.green() * 8, JewelSensor.blue() * 8, hsvValues);
        if (JewelSensor.blue() > JewelSensor.red() && JewelSensor.blue() > JewelSensor.green()) {
            JewelColor = blue;
        } else if (JewelSensor.red() > JewelSensor.blue() && JewelSensor.red() > JewelSensor.green()) {
            JewelColor = red;
        }
        else {
            JewelColor= "No Detected";
        }
        telemetry.addData(">  Jewel Color: ",JewelColor);
    }
    protected void CheckStoneColor(){
        float hsvValues[] = {0, 0, 0};

        Color.RGBToHSV(LineSensor.red() * 8, LineSensor.green() * 8, LineSensor.blue() * 8, hsvValues);
        if (LineSensor.blue() > LineSensor.red() && LineSensor.blue() > LineSensor.green()) {
            StoneColor = blue;
        } else if (LineSensor.red() > LineSensor.blue() && LineSensor.red() > LineSensor.green()) {
            StoneColor = red;
        }
        else {
            StoneColor = "No Detected";
        }
        telemetry.addData(">  Stone Color: ",StoneColor);
    }
    protected void CheckRangeDistanceCM(){
        telemetry.addData(">  Cm", "%.2f cm", RangeSensor.getDistance(DistanceUnit.CM));
    }
    protected void CheckHeading(){
        telemetry.addData(">  Z: ",GyroSensor.getIntegratedZValue());
    }
    protected void CheckAngle(){
        getAngle();
        telemetry.addData(">  Angle", lastAngles.firstAngle);
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
    protected void FourEncoders(){
        telemetry.addData("> R_M_F; ", Math.abs(R_M_F.getCurrentPosition()));
        telemetry.addData("> R_M_B: ", Math.abs(R_M_B.getCurrentPosition()));
        telemetry.addData("> L_M_F: ", Math.abs(L_M_F.getCurrentPosition()));
        telemetry.addData("> L_M_B: ", Math.abs(L_M_B.getCurrentPosition()));

    }
}
