package org.firstinspires.ftc.teamcode.RelicRecovery.Encoders.Testing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static java.lang.Thread.sleep;

/**
 * Created by Jorge on 03/11/2017.
 */
//@Autonomous(name = "Encoder4_0")
public class Encoder4_0 extends OpMode {

    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motor4;

    @Override
    public void init() {
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");
        motor3 = hardwareMap.dcMotor.get("motor3");
        motor4 = hardwareMap.dcMotor.get("motor4");
        motor3.setDirection(DcMotor.Direction.REVERSE);
        motor4.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        ForwardWithDistance(.1,5);
        Telemetry();
        //sleep(1500);
        TelemetryRotations();
        //sleep(1500);
    }

    public void ForwardWithDistance(double DrivePower,int rotations){
     ResetEncoders();
     SetTargetPosition(rotations*1140);
     RunToPosition();
     DriveForward(DrivePower);
     WhileBusyMotors();
     StopDriving();
     RunUsingEncoders();
    }
    private void ResetEncoders(){
        motor1.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor2.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor3.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor4.setMode(DcMotor.RunMode.RESET_ENCODERS);
    }
    private void SetTargetPosition(int distance1){
        motor1.setTargetPosition(distance1);
        motor2.setTargetPosition(distance1);
        motor3.setTargetPosition(distance1);
        motor4.setTargetPosition(distance1);
    }
    private void RunToPosition(){
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void RunUsingEncoders(){
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void DriveForward(double power){
        motor1.setPower(power);
        motor2.setPower(power);
        motor3.setPower(power);
        motor4.setPower(power);
    }
    private void WhileBusyMotors(){
        while (motor1.isBusy() && motor2.isBusy() && motor3.isBusy() && motor4.isBusy()) {
            //NOTHING
        }
    }
    private void StopDriving(){
        DriveForward(0);
    }
    private void Telemetry(){
        telemetry.addData("Distance M1",motor1.getCurrentPosition());
        telemetry.addData("Distance M2",motor2.getCurrentPosition());
        telemetry.addData("Distance M3",motor3.getCurrentPosition());
        telemetry.addData("Distance M4",motor4.getCurrentPosition());
    }
    private void TelemetryRotations(){
        telemetry.addData("Rotations M1",motor1.getCurrentPosition()/1140);
        telemetry.addData("Rotations M2",motor2.getCurrentPosition()/1140);
        telemetry.addData("Rotations M3",motor3.getCurrentPosition()/1140);
        telemetry.addData("Rotations M4",motor4.getCurrentPosition()/1140);
    }
}