package org.firstinspires.ftc.teamcode.RelicRecovery.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Jorge on 14/11/2017.
 */
//@TeleOp(name ="TeleOpRR_Proto",group = "TeleOp")
public class TeleOpRR_Proto extends OpMode {

    //MOTORS
    public DcMotor L_M_F = null;
    public DcMotor L_M_B = null;
    public DcMotor R_M_F = null;
    public DcMotor R_M_B = null;
    public DcMotor Lifter = null;
    public DcMotor RelicArm = null;
    public DcMotor GlyphGrabberR = null;
    public DcMotor GlyphGrabberL = null;

    //SERVOS
    public Servo R_S_J = null;
    public Servo L_S_J = null;
    public Servo R_C = null;
    public Servo R_A = null;


    @Override
    public void init() {
        L_M_F = hardwareMap.dcMotor.get("L_M_F");
        L_M_B = hardwareMap.dcMotor.get("L_M_B");
        R_M_F = hardwareMap.dcMotor.get("R_M_F");
        R_M_B = hardwareMap.dcMotor.get("R_M_B");
        Lifter = hardwareMap.dcMotor.get("Lifter");
        RelicArm = hardwareMap.dcMotor.get("RelicArm");
        GlyphGrabberL = hardwareMap.dcMotor.get("GlyphGrabberL");
        GlyphGrabberR = hardwareMap.dcMotor.get("GlyphGrabberR");

        L_M_F.setDirection(DcMotor.Direction.REVERSE);
        L_M_B.setDirection(DcMotor.Direction.REVERSE);
        Lifter.setDirection(DcMotor.Direction.REVERSE);
        GlyphGrabberL.setDirection(DcMotor.Direction.REVERSE);

        R_S_J = hardwareMap.servo.get("R_S_J");
        L_S_J = hardwareMap.servo.get("L_S_J");
        R_C = hardwareMap.servo.get("R_C");
        R_A = hardwareMap.servo.get("R_A");

        R_C.setPosition(0);
        R_A.setPosition(.4);
        R_S_J.setPosition(.5);
        L_S_J.setPosition(1);
    }

    @Override
    public void loop() {
        R_M_F.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        R_M_B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        L_M_F.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        L_M_B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //WHEELS
        R_M_F.setPower(gamepad1.right_stick_y);
        R_M_B.setPower(gamepad1.right_stick_y);
        L_M_F.setPower(gamepad1.left_stick_y);
        L_M_B.setPower(gamepad1.left_stick_y);

        if (gamepad2.right_stick_y > 0 && gamepad2.right_stick_y < 0) {
            Lifter.setPower(gamepad1.right_stick_y);
        } else {
            Lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (gamepad1.left_bumper) {
            L_S_J.setPosition(.6);
        }
        else if (gamepad1.right_bumper){
            R_S_J.setPosition(0);
        }
        else{
            R_S_J.setPosition(.5);
            L_S_J.setPosition(1);
        }

        RelicArm.setPower(gamepad2.left_stick_y);
        if (gamepad2.a){

        }
    }
}