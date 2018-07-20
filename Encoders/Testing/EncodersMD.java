package org.firstinspires.ftc.teamcode.RelicRecovery.Encoders.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * Created by Jorge on 02/11/2017.
 */
//@Autonomous(name = "EnocderMD")
public class EncodersMD extends LinearOpMode {


    DcMotor motor1;
    int target = 500, startPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        startPosition = motor1.getCurrentPosition();

        motor1.setPower(.1);
        motor1.setTargetPosition(target + startPosition);

        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Text", "Run to Position");
            telemetry.addData("Power", motor1.getPower());
            telemetry.addData("Position", motor1.getCurrentPosition() - startPosition);
            if (motor1.isBusy()){
                telemetry.addData("Status", "Busy");
            }
            else {
                telemetry.addData("Status", "Free");
                sleep(500);
            }


        }

    }
}
