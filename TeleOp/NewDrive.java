package org.firstinspires.ftc.teamcode.RelicRecovery.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Jorge on 16/01/2018.
 */
//@TeleOp(name="NewDrive")
public class NewDrive extends OpMode {

    //WHEELS
    protected DcMotor lf;
    protected DcMotor lb;
    protected DcMotor rf;
    protected DcMotor rb;
    DcMotor arm;
    Servo hand;

    @Override
    public void init() {

        lf = hardwareMap.dcMotor.get("lf");
        lb = hardwareMap.dcMotor.get("lb");
        rf = hardwareMap.dcMotor.get("rf");
        rb = hardwareMap.dcMotor.get("rb");
        arm = hardwareMap.dcMotor.get("arm");
        hand = hardwareMap.servo.get("hand");

        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        rf.setPower(gamepad1.right_stick_y);
        rb.setPower(gamepad1.right_stick_y);
        lf.setPower(gamepad1.left_stick_y);
        lb.setPower(gamepad1.left_stick_y);
        if (gamepad1.a){
            arm.setPower(.2);
        }
        else if (gamepad1.y){
            arm.setPower(-.5);
        }
        else{
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            arm.setPower(0);
        }

        if (gamepad1.right_bumper){
            hand.setPosition(0);
        }
        else{
            hand.setPosition(1);
        }
       /* //RIGHT TURN
        if (gamepad1.left_stick_x>0){
            rf.setPower(-gamepad1.left_stick_x);
            rb.setPower(-gamepad1.left_stick_x);
            lf.setPower(gamepad1.left_stick_x);
            lb.setPower(gamepad1.left_stick_x);
        }
        //LEFT TURN
        if (gamepad1.left_stick_x<0){
            rf.setPower(gamepad1.left_stick_x);
            rb.setPower(gamepad1.left_stick_x);
            lf.setPower(-gamepad1.left_stick_x);
            lb.setPower(-gamepad1.left_stick_x);
        }
        //STRAIGHT AND REVERSE
        if (gamepad1.right_stick_y>0 || gamepad1.right_stick_y<0){
            rf.setPower(gamepad1.right_stick_y);
            rb.setPower(gamepad1.right_stick_y);
            lf.setPower(gamepad1.right_stick_y);
            lb.setPower(gamepad1.right_stick_y);
        }
        //HORIZONTAL RIGHT
        if (gamepad1.right_stick_x>0){
            rf.setPower(-gamepad1.right_stick_x);
            rb.setPower(gamepad1.right_stick_x);
            lf.setPower(gamepad1.right_stick_x);
            lb.setPower(-gamepad1.right_stick_x);
        }
        //HORIZONTAL LEFT
        if (gamepad1.right_stick_x<0){
            rf.setPower(gamepad1.right_stick_x);
            rb.setPower(-gamepad1.right_stick_x);
            lf.setPower(-gamepad1.right_stick_x);
            lb.setPower(gamepad1.right_stick_x);
        }*/
    }
}