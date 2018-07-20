package org.firstinspires.ftc.teamcode.RelicRecovery.Encoders.Testing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static java.lang.Thread.sleep;

/**
 * Created by Jorge on 03/11/2017.
 */
//Autonomous(name = "Isaac1_0")
public class Isaac1_0 extends LinearOpMode {

    private DcMotor motor1;

    @Override
    public void runOpMode() throws InterruptedException {

        motor1 = hardwareMap.dcMotor.get("motor1");

        while(opModeIsActive()){
            telemetry.addData("Starting Telemetry PARO FUNCIONA CABRON ", "");
            telemetry.addData("Position: ", motor1.getCurrentPosition());
        }

            waitForStart();
        telemetry.addData("Starting Telemetry PARO FUNCIONA CABRON ", "");
        telemetry.addData("Position: ", motor1.getCurrentPosition());
          //  ForwardWithDistance(.1,5);
           // Telemetry();
            //sleep(1500);
            //TelemetryRotations();
            sleep(1500000);
        }



    public void ForwardWithDistance(double DrivePower,int rotations){
        ResetEncoders();
        telemetry.addLine("ResetEnocders DONE");
        RunToPosition();
        SetTargetPosition(rotations*1140);
        telemetry.addLine("SetRargaetPosition DONE");
        DriveForward(DrivePower);
        telemetry.addLine("RunToPosition and Drive Forward DONE");
        WhileBusyMotors();
        StopDriving();
        RunUsingEncoders();
    }
    private void ResetEncoders(){
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void SetTargetPosition(int distance1){
        motor1.setTargetPosition(distance1);
    }
    private void RunToPosition(){
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void RunUsingEncoders(){
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void DriveForward(double power){
        motor1.setPower(power);
    }
    private void WhileBusyMotors(){
        while (motor1.isBusy()) {
            idle(); // prevent unnecessary cpu usage, since we don't need to constantly poll the motors
            if (!opModeIsActive() || Thread.currentThread().isInterrupted()) {
                try {
                    throw new InterruptedException(); // this is the simplest method to not crash your robot controller, due to this loop
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }
    private void StopDriving(){
        DriveForward(0);
    }
    private void Telemetry(){
        telemetry.addData("Distance M1",motor1.getCurrentPosition());
        telemetry.update();
    }
    private void TelemetryRotations(){
        telemetry.addData("Rotations M1",motor1.getCurrentPosition()/1140);
        telemetry.update();
    }

    /*
    *  public void ForwardWithDistance(double DrivePower,int rotations){
        ResetEncoders();
        telemetry.addLine("ResetEnocders DONE");
        SetTargetPosition(rotations*1140);
        telemetry.addLine("SetRargaetPosition DONE");
        RunToPosition();
        DriveForward(DrivePower);
        telemetry.addLine("RunToPosition and Drive Forward DONE");
        WhileBusyMotors();
        StopDriving();
        RunUsingEncoders();
    }
    private void ResetEncoders(){
        motor1.setMode(DcMotor.RunMode.RESET_ENCODERS);
    }
    private void SetTargetPosition(int distance1){
        motor1.setTargetPosition(distance1);
    }
    private void RunToPosition(){
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void RunUsingEncoders(){
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void DriveForward(double power){
        motor1.setPower(power);
    }
    private void WhileBusyMotors(){
        while (motor1.isBusy()) {
            //NOTHING
        }
    }
    private void StopDriving(){
        DriveForward(0);
    }
    private void Telemetry(){
        telemetry.addData("Distance M1",motor1.getCurrentPosition());
    }
    private void TelemetryRotations(){
        telemetry.addData("Rotations M1",motor1.getCurrentPosition()/1140);
    }
    * */
}