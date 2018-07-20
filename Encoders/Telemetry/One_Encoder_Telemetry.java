package org.firstinspires.ftc.teamcode.RelicRecovery.Encoders.Telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Jorge on 01/12/2017.
 */
//@TeleOp(name ="One_Encoder_Telemetry",group = "TeleOp")
public class One_Encoder_Telemetry extends LinearOpMode {
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
        MotorsForward();

        waitForStart();
        ResetEncoders();
        while (opModeIsActive()){FourEncodersTelemetry();}
    }
    private void ForwardToPosition(double power, int rotations){
        /*R_M_F.setMode(DcMotor.RunMode.RESET_ENCODERS);
        Left_motors.setMode(DcMotor.RunMode.RESET_ENCODERS);*/
        R_M_F.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        R_M_F.setTargetPosition(rotations*1120);

        R_M_F.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Forward(power);

        while(R_M_F.isBusy()){
            telemetry.addLine("Encoders working");
            telemetry.update();
        }

        StopDriving();
        R_M_F.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void Forward(double power){
        L_M_F.setPower(power);
        R_M_F.setPower(power);
        L_M_B.setPower(power);
        R_M_B.setPower(power);
    }
    private void Reverse(double power){
        Forward(-power);
    }
    private void StopDriving(){
        Forward(0);
    }
    public void ResetEncoders(){
        R_M_F.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R_M_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L_M_F.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L_M_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void MotorsForward(){
        R_M_F.setDirection(DcMotor.Direction.FORWARD);
        R_M_B.setDirection(DcMotor.Direction.REVERSE);
        L_M_F.setDirection(DcMotor.Direction.REVERSE);
        L_M_B.setDirection(DcMotor.Direction.REVERSE);
    }
    public void MotorsReverse(){
        R_M_F.setDirection(DcMotor.Direction.REVERSE);
        R_M_B.setDirection(DcMotor.Direction.FORWARD);
        L_M_F.setDirection(DcMotor.Direction.FORWARD);
        L_M_B.setDirection(DcMotor.Direction.FORWARD);
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
