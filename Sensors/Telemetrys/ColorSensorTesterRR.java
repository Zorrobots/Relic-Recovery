package org.firstinspires.ftc.teamcode.RelicRecovery.Sensors.Telemetrys;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by Jorge on 15/12/2016.
 */
@TeleOp(name ="ColorSensorTesterRR")
public class ColorSensorTesterRR extends OpMode {

    //BUS 0 PORT 1
    String white = "white";
    String blue = "blue";
    String red = "red";
    String green = "green";
    String black = "black";

    ColorSensor JewelSensor;

    @Override
    public void init() {

        JewelSensor = hardwareMap.colorSensor.get("JewelSensor");
        //ColorSensor.setI2cAddress(I2cAddr.create7bit(0x26));//4c
        JewelSensor.enableLed(true);

    }

    @Override
    public void loop() {

        JewelSensor.enableLed(true);

        float hsvValues[] = {0,0,0};


        Color.RGBToHSV(JewelSensor.red()*8, JewelSensor.green()*8,JewelSensor.blue()*8,hsvValues);
        telemetry.addData("Blue", JewelSensor.blue());
        telemetry.addData("Red", JewelSensor.red());
        telemetry.addData("Green", JewelSensor.green());
    }
}
