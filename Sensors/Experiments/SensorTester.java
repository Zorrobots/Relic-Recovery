package org.firstinspires.ftc.teamcode.RelicRecovery.Sensors.Experiments;


import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

/**
 * Created by Eli on 26/10/2017.
 */
//@TeleOp(name = "SensorTester")
public class SensorTester extends LinearOpMode {

    private ColorSensor ColorSensor;
    private DistanceSensor DistanceSensor;

    @Override
    public void runOpMode() throws InterruptedException {

        ColorSensor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        DistanceSensor = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");


        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double SCALE_FACTOR = 255;
        
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        waitForStart();
        while (opModeIsActive()) {
            Color.RGBToHSV((int) (ColorSensor.red() * SCALE_FACTOR),
                    (int) (ColorSensor.green() * SCALE_FACTOR),
                    (int) (ColorSensor.blue() * SCALE_FACTOR),
                    hsvValues);

            telemetry.addData("Distance (cm)",
                    String.format(Locale.US, "%.02f", DistanceSensor.getDistance(DistanceUnit.CM)));
            telemetry.addData("Alpha", ColorSensor.alpha());
            telemetry.addData("Red  ", ColorSensor.red());
            telemetry.addData("Green", ColorSensor.green());
            telemetry.addData("Blue ", ColorSensor.blue());
            telemetry.addData("Hue", hsvValues[0]);

            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });

            telemetry.update();
        }
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });
    }
}