package org.firstinspires.ftc.teamcode.RelicRecovery.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Jorge on 14/12/2017.
 */
//@TeleOp(name="LifterTester")
public class LifterTester extends OpMode {
    DcMotor R_M_F;
    DcMotor R_M_B;
    DcMotor L_M_F;
    DcMotor L_M_B;
    DcMotor Lifter;
    Servo RightJServo;
    Servo LeftJServo;
    Servo LeftCServo;
    Servo RightCServo;

    @Override
    public void init() {
       /* L_M_F = hardwareMap.dcMotor.get("L_M_F");
        L_M_F = hardwareMap.dcMotor.get("L_M_B");
        R_M_F = hardwareMap.dcMotor.get("R_M_F");
        R_M_F = hardwareMap.dcMotor.get("R_M_B");*/
        Lifter = hardwareMap.dcMotor.get("Lifter");
       // RightJServo = hardwareMap.servo.get("RightJServo");
        //LeftJServo = hardwareMap.servo.get("LeftJServo");
        LeftCServo = hardwareMap.servo.get("LeftCServo");
        RightCServo = hardwareMap.servo.get("RightCServo");
        Lifter.setDirection(DcMotor.Direction.REVERSE);
    }
    @Override
    public void loop() {
      /*  R_M_F.setPower(gamepad1.right_stick_y);
        R_M_B.setPower(gamepad1.right_stick_y);
        L_M_F.setPower(gamepad1.left_stick_y);
        L_M_B.setPower(gamepad1.left_stick_y);
*/
        if(gamepad1.y){
            Lifter.setPower(1);
        }
        else if(gamepad1.a){
            Lifter.setPower(-1);
        }
        else{
            Lifter.setPower(0);
        }

        if(gamepad1.right_bumper){
            LeftCServo.setPosition(1);
            RightCServo.setPosition(0);
        }
        else{
            LeftCServo.setPosition(.5);
            RightCServo.setPosition(.5);
        }

        /*if (gamepad1.x){
            LeftJServo.setPosition(1);
        }
        else if(gamepad1.b){
            RightJServo.setPosition(1);
        }
        else{
            RightJServo.setPosition(0);
            LeftJServo.setPosition(0);
        }*/


    }
}
