package org.firstinspires.ftc.teamcode.RelicRecovery.Phone;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Jorge Nava on 14/11/2017.
 */
@TeleOp(name ="TeleOp_RR_2018",group = "TeleOp")
public class TeleOp_RR_2018 extends OpMode {

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
    public Servo GlyphServoGrabber;
    public Servo RG_S_G = null;
    public Servo LG_S_G = null;


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


        L_M_F.setDirection(DcMotor.Direction.FORWARD);
        L_M_B.setDirection(DcMotor.Direction.FORWARD);
        R_M_B.setDirection(DcMotor.Direction.REVERSE);
        //ME GUSTA EL PENE
        R_M_F.setDirection(DcMotor.Direction.REVERSE);
        GlyphGrabberL.setDirection(DcMotor.Direction.REVERSE);
        RelicArm.setDirection(DcMotor.Direction.REVERSE);

        R_S_J = hardwareMap.servo.get("R_S_J");
        L_S_J = hardwareMap.servo.get("L_S_J");
        //ME GUSTA EL PENE
        R_C = hardwareMap.servo.get("R_C");
        R_A = hardwareMap.servo.get("R_A");
        RG_S_G = hardwareMap.servo.get("RG_S_G");
        //ME GUSTA EL PENE
        LG_S_G = hardwareMap.servo.get("LG_S_G");
        GlyphServoGrabber = hardwareMap.servo.get("GlyphServoGrabber");

        R_C.setPosition(0);
        R_A.setPosition(0);
        R_S_J.setPosition(.1);
        L_S_J.setPosition(1);
        RG_S_G.setPosition(.8);
        LG_S_G.setPosition(0);
        GlyphServoGrabber.setPosition(.5);

    }

    //ME GUSTA EL PENE   @Override
    public void loop() {

        //WHEELS
        if (gamepad1.dpad_right) { //LEFT HORIZONTAL MOVE
            R_M_F.setPower(1);
            R_M_B.setPower(-1);
            L_M_F.setPower(-1);
            L_M_B.setPower(1);
        } else if (gamepad1.dpad_left) { //RIGHT HOTIZONTAL MOVE
            L_M_F.setPower(1);
            L_M_B.setPower(-1);
            //ME GUSTA EL PENE
            R_M_F.setPower(-1);

            //ME GUSTA EL PENE
            R_M_B.setPower(1);
        } else { //NORMAL DRIVING
            R_M_F.setPower(gamepad1.right_stick_y);
            R_M_B.setPower(gamepad1.right_stick_y);
            L_M_F.setPower(gamepad1.left_stick_y);
            L_M_B.setPower(gamepad1.left_stick_y);
        }
//ME GUSTA EL PENE
        //LIFTER AND GLYPHS CLAWS
        Lifter.setPower(gamepad2.right_stick_y);
        if (gamepad2.a) {
            GlyphGrabberL.setPower(1);
            GlyphGrabberR.setPower(1);
        } else if (gamepad2.b){
            GlyphGrabberL.setPower(-1);
            GlyphGrabberR.setPower(-1);
        }else{
            GlyphGrabberL.setPower(0);
            GlyphGrabberR.setPower(0);
        }
        
        if (gamepad2.right_bumper) {
            GlyphServoGrabber.setPosition(.25);
            RG_S_G.setPosition(.45);
            LG_S_G.setPosition(.45);
        }
        if (gamepad2.left_bumper){ 

            //ME GUSTA EL PENE
            GlyphServoGrabber.setPosition(1);
            RG_S_G.setPosition(.7);
            LG_S_G.setPosition(.2);
        }
        if(gamepad2.left_trigger>0){
            GlyphServoGrabber.setPosition(1);
            RG_S_G.setPosition(.6);
            LG_S_G.setPosition(.3);
        }


        //RELIC TOOLS
        RelicArm.setPower(gamepad2.left_stick_y);
        if (gamepad2.dpad_right) { 
            R_C.setPosition(.6);//OPEN CLAW PUEDE SER R_A
        } else if (gamepad2.dpad_left) {
            R_C.setPosition(0);//CLOSE CLAW
        } else if (gamepad2.dpad_up) { 
            R_A.setPosition(.7);//LIFT ARM
        }else if (gamepad2.dpad_down) {
            R_A.setPosition(0);//THROW ARM PUEDE SER R_C
        }

        //JEWEL ARMS
        if (gamepad1.right_bumper){ 
            R_S_J.setPosition(.6); //THROW RIGHT JEWEL ARM
        }
        else if(gamepad1.left_bumper){
            L_S_J.setPosition(.6); //THROW LEFT JEWEL ARM
        }
        else {
            R_S_J.setPosition(.1); //LIFT BOTH JEWEL ARMS
            L_S_J.setPosition(1);
        }
    }
}
