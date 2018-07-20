package org.firstinspires.ftc.teamcode.RelicRecovery.Sensors.Telemetrys;

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/**
 * Created by Jorge on 23/11/2017.
 */
//@Autonomous(name="GyroTelemetry")
public class GyroTelemetry extends LinearOpMode {

    protected ModernRoboticsI2cGyro GyroSensor;
    protected ModernRoboticsI2cGyro GyroS;


    @Override
    public void runOpMode() throws InterruptedException {

        GyroSensor = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("GyroSensor");
        GyroS = (ModernRoboticsI2cGyro) GyroSensor;
    while (opModeIsActive()){
        telemetry.addData("0 heading:",GyroSensor.getHeading());
        telemetry.addData("1 integrated z:",GyroSensor.getIntegratedZValue());
        telemetry.addData("2 x raw:",GyroSensor.rawX());
        telemetry.addData("3 y raw:", GyroSensor.rawY());
        telemetry.addData("4 z raw:", GyroSensor.rawZ());
        telemetry.update();
        GyroSensor.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

    }
    }
    public void CheckGyroValues() {
        telemetry.addData("0", "Heading %03d", GyroSensor.getHeading());
        telemetry.addData("1", "Int. Ang. %03d", GyroSensor.getIntegratedZValue());
        telemetry.addData("2", "X av. %03d", GyroSensor.rawX());
        telemetry.addData("3", "Y av. %03d", GyroSensor.rawY());
        telemetry.addData("4", "Z av. %03d", GyroSensor.rawZ());
        telemetry.update();
    }
}
