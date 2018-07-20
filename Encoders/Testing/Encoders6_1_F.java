package org.firstinspires.ftc.teamcode.RelicRecovery.Encoders.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Jorge on 01/12/2017.
 */
//@TeleOp(name ="Encoders6_1_F",group = "TeleOp")
public class Encoders6_1_F extends LinearOpMode {
    private DcMotor R_M_B = null;


    /*
    *   static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    * */

    @Override
    public void runOpMode() throws InterruptedException {
        R_M_B = hardwareMap.dcMotor.get("R_M_B");

        R_M_B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //L_M_B REVERSE WITH MATH JUST CHANGE DIRECTION FROM REVERSE-FRON-REVERSE
        //L_M_F SAME AS LMB
        //R_M_B FORWARD
        //R_M_B
        //ANDYMARK TICKS PER REVOLUTIONS 1120

        waitForStart();
        telemetry.addLine("Starting encoders");
        telemetry.update();
        ForwardDistance(1,5);
        ReverseDistance(1,5);
        telemetry.addLine("Encoders Finished");
        telemetry.update();
    }

    protected void ForwardDistance(double power, int rotations){
        R_M_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int Target = (rotations*1120);
        telemetry.addData("> Target: ",Target);
        telemetry.update();
        sleep(1000);
        R_M_B.setPower(power);
        while(Math.abs(R_M_B.getCurrentPosition())<Target){
            R_M_B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addLine("Encoders working");
            telemetry.addData("> Target: ",Target);
            telemetry.addData("> Rotation: ",R_M_B.getCurrentPosition()/1120);
            telemetry.addData("> Math Position: ",Math.abs(R_M_B.getCurrentPosition()));
            telemetry.update();
        }
        R_M_B.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    protected void ReverseDistance(double power, int rotations){
        sleep(500);
        R_M_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R_M_B.setDirection(DcMotor.Direction.REVERSE);
        sleep(500);

        int Target = (rotations*1120);
        telemetry.addData("> Target: ",Target);
        telemetry.update();
        sleep(1000);
        R_M_B.setPower(power);
        while(Math.abs(R_M_B.getCurrentPosition())<Target){
            R_M_B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addLine("Encoders working");
            telemetry.addData("> Target: ",Target);
            telemetry.addData("> Rotation: ",R_M_B.getCurrentPosition()/1120);
            telemetry.addData("> Math Position: ",Math.abs(R_M_B.getCurrentPosition()));
            telemetry.update();
        }
        R_M_B.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    private void Forward(double power){
        R_M_B.setPower(power);
    }
    private void StopDriving(){
        Forward(0);
    }
}
