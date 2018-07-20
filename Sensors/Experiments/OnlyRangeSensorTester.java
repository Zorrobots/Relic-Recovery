package org.firstinspires.ftc.teamcode.RelicRecovery.Sensors.Experiments;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Jorge on 11/12/2017.
 */
//@Autonomous(name = "OnlyrangeSensorTester", group = "Sensor")
public class OnlyRangeSensorTester extends LinearOpMode{

    public DcMotor L_M_F = null;
    public DcMotor L_M_B = null;
    public DcMotor R_M_F = null;
    public DcMotor R_M_B = null;
    ModernRoboticsI2cRangeSensor RangeSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        R_M_F = hardwareMap.dcMotor.get("R_M_F");
        R_M_B = hardwareMap.dcMotor.get("R_M_B");
        L_M_F = hardwareMap.dcMotor.get("L_M_F");
        L_M_B = hardwareMap.dcMotor.get("L_M_B");
        RangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "RangeSensor");
       waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("cm optical", "%.2f cm", RangeSensor.cmOptical());
            telemetry.addData("cm", "%.2f cm", RangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
            double distance = RangeSensor.getDistance(DistanceUnit.CM);
            double target = 15;
            while ( Math.abs(distance-target) > 3){
                Forward(.2);
            }
        }
    }
    private void Forward(double power){
        R_M_F.setPower(power);
        R_M_B.setPower(power);
        L_M_F.setPower(power);
        L_M_B.setPower(power);
    }
}
