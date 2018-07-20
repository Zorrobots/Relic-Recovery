package org.firstinspires.ftc.teamcode.RelicRecovery.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Jorge on 14/11/2017.
 */
//@TeleOp(name ="FourMotors",group = "TeleOp")
public class FourMotors extends OpMode {

    //WHEELS
    public DcMotor motor1 = null;
    public DcMotor motor3 = null;
    public DcMotor motor2 = null;
    public DcMotor motor4 = null;


    @Override
    public void init() {
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor3 = hardwareMap.dcMotor.get("motor3");
        motor4 = hardwareMap.dcMotor.get("motor4");
        motor2 = hardwareMap.dcMotor.get("motor2");

        motor3.setDirection(DcMotor.Direction.REVERSE);
        motor4.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void loop() {
        //WHEELS
        motor1.setPower(gamepad1.left_stick_y);
        motor2.setPower(gamepad1.left_stick_y);
        motor3.setPower(gamepad1.right_stick_y);
        motor4.setPower(gamepad1.right_stick_y);
    }
}
