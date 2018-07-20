package org.firstinspires.ftc.teamcode.RelicRecovery.Encoders.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Jorge on 01/12/2017.
 */
//@TeleOp(name ="Encoders8_0",group = "TeleOp")
public class Encoders8_0 extends LinearOpMode {
    protected DcMotor L_M_F = null;
    protected DcMotor L_M_B = null;
    protected DcMotor R_M_F = null;
    protected DcMotor R_M_B = null;


    @Override
    public void runOpMode() throws InterruptedException {
        L_M_F = hardwareMap.dcMotor.get("L_M_F");
        L_M_B = hardwareMap.dcMotor.get("L_M_B");
        R_M_F = hardwareMap.dcMotor.get("R_M_F");
        R_M_B = hardwareMap.dcMotor.get("R_M_B");
        R_M_F.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        R_M_B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        L_M_F.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        L_M_B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        R_M_F.setDirection(DcMotor.Direction.FORWARD);
        R_M_B.setDirection(DcMotor.Direction.REVERSE);
        L_M_F.setDirection(DcMotor.Direction.REVERSE);
        L_M_B.setDirection(DcMotor.Direction.REVERSE);
        //ANDYMARK TICKS PER REVOLUTIONS 1120

        waitForStart();
        telemetry.addLine("Starting encoders");
        telemetry.update();
        ForwardDistance(1,3.5);
        ReverseDistance(1,3.5);
        telemetry.addLine("Encoders Finished");
        telemetry.update();
    }

    protected void ForwardDistance(double power, double rotations){
        R_M_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R_M_F.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L_M_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L_M_F.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double Target = (rotations*1120);
        telemetry.addData("> Target: ",Target);
        telemetry.update();
        sleep(1000);
        R_M_F.setPower(power);
        L_M_F.setPower(power);
        R_M_B.setPower(power);
        L_M_B.setPower(power);
        while ((Math.abs(R_M_B.getCurrentPosition()) < Target && Math.abs(R_M_F.getCurrentPosition()) < Target && Math.abs(L_M_F.getCurrentPosition()) < Target && Math.abs(L_M_B.getCurrentPosition()) < Target)){
            R_M_F.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            L_M_F.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            R_M_B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            L_M_B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FourEncodersTelemetry();
            telemetry.update();
        }
        R_M_F.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        L_M_F.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        L_M_B.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        R_M_B.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    protected void ReverseDistance(double power, double rotations){
        R_M_F.setDirection(DcMotor.Direction.REVERSE);
        R_M_B.setDirection(DcMotor.Direction.FORWARD);
        L_M_F.setDirection(DcMotor.Direction.FORWARD);
        L_M_B.setDirection(DcMotor.Direction.FORWARD);

        R_M_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R_M_F.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L_M_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L_M_F.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double Target = (rotations*1120);
        telemetry.addData("> Target: ",Target);
        telemetry.update();
        sleep(1000);
        R_M_F.setPower(power);
        L_M_F.setPower(power);
        R_M_B.setPower(power);
        L_M_B.setPower(power);
        while ((Math.abs(R_M_B.getCurrentPosition()) < Target && Math.abs(R_M_F.getCurrentPosition()) < Target && Math.abs(L_M_F.getCurrentPosition()) < Target && Math.abs(L_M_B.getCurrentPosition()) < Target)){
            R_M_F.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            L_M_F.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            R_M_B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            L_M_B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FourEncodersTelemetry();
            telemetry.update();
        }
        R_M_F.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        L_M_F.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        L_M_B.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        R_M_B.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        R_M_F.setDirection(DcMotor.Direction.FORWARD);
        R_M_B.setDirection(DcMotor.Direction.REVERSE);
        L_M_F.setDirection(DcMotor.Direction.REVERSE);
        L_M_B.setDirection(DcMotor.Direction.REVERSE);
    }
    private void Forward(double power){
        R_M_B.setPower(power);
    }
    private void StopDriving(){
        Forward(0);
    }
    private void FourEncodersTelemetry(){
        telemetry.addLine("R_M_F");
        telemetry.addData("> Rotation: ", R_M_F.getCurrentPosition() / 1120);
        telemetry.addData("> Math Position: ", Math.abs(R_M_F.getCurrentPosition()));
        telemetry.addLine("R_M_B");
        telemetry.addData("> Rotation: ", R_M_B.getCurrentPosition() / 1120);
        telemetry.addData("> Math Position: ", Math.abs(R_M_B.getCurrentPosition()));
        telemetry.addLine("L_M_F");
        telemetry.addData("> Rotation: ", L_M_F.getCurrentPosition() / 1120);
        telemetry.addData("> Math Position: ", Math.abs(L_M_F.getCurrentPosition()));
        telemetry.addData("> Rotation: ", L_M_B.getCurrentPosition() / 1120);
        telemetry.addLine("L_M_B");
        telemetry.addData("> Math Position: ", Math.abs(L_M_B.getCurrentPosition()));
        telemetry.update();
    }

}
