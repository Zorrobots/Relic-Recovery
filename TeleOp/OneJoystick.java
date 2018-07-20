package org.firstinspires.ftc.teamcode.RelicRecovery.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Jorge on 04/12/2017.
 */
//@TeleOp(name ="OneJoystick",group = "TeleOp")
public class OneJoystick extends OpMode{
    //WHEELS
    public DcMotor Left_motors = null;
    public DcMotor Right_motors = null;

    @Override
    public void init() {
        Left_motors = hardwareMap.dcMotor.get("Left_motors");
        Right_motors = hardwareMap.dcMotor.get("Right_motors");

        Right_motors.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void loop() {
        //WHEELS
        double throttle = gamepad1.right_stick_x;
        double turn = gamepad1.right_stick_x;

        double leftspeed = throttle - turn;
        double rightspeed = throttle + turn;

        Left_motors.setPower(leftspeed);
        Right_motors.setPower(rightspeed);

    }
}
