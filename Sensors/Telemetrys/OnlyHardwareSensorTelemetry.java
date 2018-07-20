package org.firstinspires.ftc.teamcode.RelicRecovery.Sensors.Telemetrys;

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
/*
1cm = 1022  6cm = 60    11cm = 21
2cm = 440   7cm = 45    12cm = 17
3cm = 225   8cm = 35    13cm = 15
4cm = 130   9cm = 30    14cm = 13
5cm = 85    10cm =24    15cm = 12
 */

/**
 * Created by Jorge on 08/12/2016.
 */
//@TeleOp(name = "OnlyHardwareSensorTelemetry")
public class OnlyHardwareSensorTelemetry extends LinearOpMode {
    private ColorSensor ColorSensor;
    private ColorSensor ColorSensor2;
    private GyroSensor GyroSensor;
    public ModernRoboticsI2cGyro GyroS;

    /*
     * Both  3c I2C address CS B1 P0 & CS2 B2 P0 (Worked)
     * Both  3c I2C address CS B1 P0 & CS2 B2 P0 Declarement on HardwareMap
     * CS 3c B1 P0 & CS2 4c B2 P0
     * CS 3c B1 P0 & CS2 4c B2 P0 Declarement on HardwareMap*
     */


    @Override
    public void runOpMode() throws InterruptedException {
        ColorSensor = hardwareMap.colorSensor.get("ColorSensor");
        //ColorSensor.setI2cAddress(I2cAddr.create7bit(0x26));//4c 0x26
        ColorSensor2 = hardwareMap.colorSensor.get("ColorSensor2");
        //ColorSensor2.setI2cAddress(I2cAddr.create7bit(0x1e));//3c
        GyroSensor = hardwareMap.gyroSensor.get("GyroSensor");
        GyroS = (ModernRoboticsI2cGyro)GyroSensor;
        ColorSensor2.enableLed(false);
        ColorSensor.enableLed(false);
/*
        //BEFORE WE PRESS START GYRO SENSOR NEEDS TO BE CALIBRATED
        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        GyroSensor.calibrate();

        //WHILE START ISNT PRESSED GIVE SOME TIME TO CALIBRATE
        while (!isStopRequested() && GyroSensor.isCalibrating())  {
            sleep(500);
            idle();
        }

        //SEND CALIBRATION BY TELEMETRY
        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();
        sleep(200);
        while (!opModeIsActive()){
            telemetry.addData("Grados: ", GyroS.getIntegratedZValue());
            telemetry.update();
        }
*/
        waitForStart();
        while (opModeIsActive()){
            telemetry.addLine("ColorSensor:");
            ColorTelemtry();
            telemetry.addLine("ColorSensor2:");
            Color2Telemtry();
            telemetry.addLine("GyroSensor:");
            GyroTelemetry();
            telemetry.update();
        }

    }

    public void ColorTelemtry (){
        ColorSensor.enableLed(true);

        float hsvValues[] = {0,0,0};


        Color.RGBToHSV(ColorSensor.red()*8, ColorSensor.green()*8,ColorSensor.blue()*8,hsvValues);
        telemetry.addData("Azul", ColorSensor.blue());
        telemetry.addData("Rojo", ColorSensor.red());
        telemetry.addData("Verde", ColorSensor.green());
    }
    public void Color2Telemtry (){
        ColorSensor.enableLed(true);

        float hsvValues[] = {0,0,0};


        Color.RGBToHSV(ColorSensor2.red()*8, ColorSensor2.green()*8,ColorSensor2.blue()*8,hsvValues);
        telemetry.addData("Azul", ColorSensor2.blue());
        telemetry.addData("Rojo", ColorSensor2.red());
        telemetry.addData("Verde", ColorSensor2.green());
    }


    public void GyroTelemetry(){

        telemetry.addData("Heading: ", GyroSensor.getHeading());

    }
}