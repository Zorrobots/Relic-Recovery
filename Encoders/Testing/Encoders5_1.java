package org.firstinspires.ftc.teamcode.RelicRecovery.Encoders.Testing;

import android.text.style.ForegroundColorSpan;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.sql.RowId;

/**
 * Created by Jorge on 01/12/2017.
 PROBAR TELEOP PROTO
 ORGANIZAR Y CONECTAR LO FALTANTE
 PONER SENSORES
 PROBAR SENSORES
 PROBAR IMU
 RESOLVER PROBLEMA DE ENCODERS
 HACER AUTONOMOS */
//@TeleOp(name ="Encoders5_1",group = "TeleOp")
public class Encoders5_1 extends LinearOpMode {
    private DcMotor R_M_F = null;
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
        R_M_F = hardwareMap.dcMotor.get("R_M_F");
        R_M_B.setDirection(DcMotor.Direction.REVERSE);
        // R_M_B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //ANDYMARK TICKS PER REVOLUTIONS 1120

        waitForStart();
        telemetry.addLine("Starting encoders");
        telemetry.update();
        // ForwardToPosition(1, 2);
        ForwardDistance(1,5);
       // ReverseDistance(1, 5);
        telemetry.addLine("Encoders Finished");
        telemetry.update();
    }

    protected void ForwardDistance(double power, int rotations){
        R_M_F.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int Target = (rotations*1120);
        telemetry.addData("> Target: ",Target);
        telemetry.update();
        sleep(1000);
        R_M_F.setPower(power);
        while(R_M_F.getCurrentPosition()<Target){
            telemetry.addLine("Encoders working");
            telemetry.addData("> Target: ",Target);
            telemetry.addData("> CURR: ",R_M_F.getCurrentPosition()/1120);
            telemetry.addData("> ABS: ",Math.abs(R_M_F.getCurrentPosition()));
            telemetry.update();
        }
        StopDriving();
    }
    protected void ReverseDistance(double power, int rotations){
        R_M_F.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R_M_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int Target = (rotations*1120)*-1;
        telemetry.addData("> Target: ",Target);
        telemetry.update();
        sleep(1000);
        R_M_F.setPower(power);
        R_M_B.setPower(power);
        while((R_M_F.getCurrentPosition()>Target)&&(R_M_B.getCurrentPosition())>Target){
            R_M_B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            R_M_B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addLine("Encoders working");
            telemetry.addData("> Target: ",Target);
            telemetry.addData("> Position: ",R_M_F.getCurrentPosition()/1120);
            telemetry.addData("> Math Position: ",Math.abs(R_M_F.getCurrentPosition()));
            telemetry.addData("> Position: ",R_M_B.getCurrentPosition()/1120);
            telemetry.addData("> Math Position: ",Math.abs(R_M_B.getCurrentPosition()));
            telemetry.update();
        }
        StopDriving();
    }
    private void Forward(double power){
        R_M_F.setPower(power);
        R_M_B.setPower(power);
    }
    private void Reverse(double power){
        Forward(-power);
    }
    private void StopDriving(){
        Forward(0);
    }
}
