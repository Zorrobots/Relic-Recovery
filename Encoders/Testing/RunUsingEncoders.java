package org.firstinspires.ftc.teamcode.RelicRecovery.Encoders.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Jorge on 02/11/2017.
 */
//@Autonomous(name="RunUsing Encoders")
//@Disabled
public class RunUsingEncoders extends OpMode {

    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motor4;

    @Override
    public void init() {
     //   motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");
   //     motor3 = hardwareMap.dcMotor.get("motor3");
 //       motor4 = hardwareMap.dcMotor.get("motor4");
       // motor4.setDirection(DcMotor.Direction.REVERSE);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
    motor2.setPower(1);
        int position = motor2.getCurrentPosition();
        telemetry.addData("Encoder Position", position);

    }
}
