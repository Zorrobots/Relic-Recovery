package org.firstinspires.ftc.teamcode.RelicRecovery.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Jorge on 14/12/2017.
 */
//@TeleOp(name="MecanumWheels")
public class MecanumWheels extends OpMode {


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

        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        L_M_F.setPower(v1);
        R_M_F.setPower(v2);
        L_M_B.setPower(v3);
        R_M_B.setPower(v4);
    }
}
