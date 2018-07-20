package org.firstinspires.ftc.teamcode.RelicRecovery.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Jorge on 14/12/2017.
 */
//@TeleOp(name="MecanumWheels2")
public class MecanumWheels2 extends OpMode {

    //WHEELS
    protected DcMotor L_M_F;
    protected DcMotor L_M_B;
    protected DcMotor R_M_F;
    protected DcMotor R_M_B;

    @Override
    public void init() {

        L_M_F = hardwareMap.dcMotor.get("L_M_F");
        L_M_B = hardwareMap.dcMotor.get("L_M_B");
        R_M_F = hardwareMap.dcMotor.get("R_M_F");
        R_M_B = hardwareMap.dcMotor.get("R_M_B");

        L_M_F.setDirection(DcMotor.Direction.REVERSE);
        L_M_B.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {

        R_M_F.setPower(gamepad1.right_stick_y);
        R_M_B.setPower(gamepad1.right_stick_y);
        L_M_F.setPower(gamepad1.left_stick_y);
        L_M_B.setPower(gamepad1.left_stick_y);


        R_M_F.setPower(gamepad1.right_trigger);

        R_M_B.setPower(-gamepad1.right_trigger);
        L_M_F.setPower(-gamepad1.right_trigger);
        L_M_B.setPower(gamepad1.right_trigger);

        R_M_F.setPower(-gamepad1.left_trigger);
        R_M_B.setPower(gamepad1.left_trigger);
        L_M_F.setPower(gamepad1.left_trigger);
        L_M_B.setPower(-gamepad1.left_trigger);
    }
}
