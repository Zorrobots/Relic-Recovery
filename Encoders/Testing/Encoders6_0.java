package org.firstinspires.ftc.teamcode.RelicRecovery.Encoders.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Jorge on 01/12/2017.
 */
//@TeleOp(name ="Encoders6_0",group = "TeleOp")
public class Encoders6_0 extends LinearOpMode {
    private DcMotor R_M_F = null;
    private DcMotor R_M_B = null;
    private DcMotor L_M_F = null;
    private DcMotor L_M_B = null;

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
        R_M_F = hardwareMap.dcMotor.get("R_M_F");
        R_M_B = hardwareMap.dcMotor.get("R_M_B");
        R_M_F.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        R_M_B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       /*R_M_F.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(500);

        int Target = rotations*1120*-1;
        telemetry.addData("> Target: ",Target);
        telemetry.update();
        sleep(1000);

        R_M_F.setTargetPosition(Target);

        R_M_F.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        R_M_F.setPower(power);
        while(R_M_F.isBusy()){
            telemetry.addLine("Encoders working");
            telemetry.addData("R_M_F: ",R_M_F.getCurrentPosition()/1120);
            telemetry.update();
        }
        R_M_F.setPower(0);*/
        //ANDYMARK TICKS PER REVOLUTIONS 1120

        waitForStart();
        telemetry.addLine("Starting encoders");
        telemetry.update();
        ForwardDistance(1,3);
        ReverseDistance(1,3);
        telemetry.addLine("Encoders Finished");
        telemetry.update();
    }
    protected void ForwardDistance(double power, int rotations){
        R_M_F.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R_M_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int Target = rotations*1120;
        telemetry.addData("> Target: ",Target);
        telemetry.update();
        sleep(1000);

        R_M_F.setTargetPosition(Target);
        R_M_B.setTargetPosition(Target);

        telemetry.addLine("SET TARGET PASSED");


        R_M_F.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        R_M_B.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addLine("RUN TO POS PASSED");

        Forward(power);
        while(R_M_F.isBusy()){
            telemetry.addLine("Encoders working");
            telemetry.addData("R_M_F: ",R_M_F.getCurrentPosition()/1120);
            telemetry.update();
        }
        StopDriving();
        R_M_F.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        R_M_B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    protected void ReverseDistance(double power, int rotations){
        R_M_F.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        R_M_F.setDirection(DcMotor.Direction.REVERSE);


        int Target = rotations*1120*-1;
        telemetry.addData("> Target: ",Target);
        telemetry.update();
        sleep(1000);

        R_M_F.setTargetPosition(Target);

        R_M_F.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Forward(power);
        while(R_M_F.isBusy()){
            telemetry.addLine("Encoders working");
            telemetry.addData("R_M_F: ",R_M_F.getCurrentPosition()/1120);
            telemetry.update();
        }
        StopDriving();
        R_M_F.setDirection(DcMotor.Direction.FORWARD);
    }
    private void Forward(double power){
        R_M_F.setPower(power);
        R_M_B.setPower(power);
    }
    protected void DirectionREVERSE(){
        R_M_B.setDirection(DcMotor.Direction.REVERSE);
        R_M_F.setDirection(DcMotor.Direction.REVERSE);
        L_M_B.setDirection(DcMotor.Direction.FORWARD);
        L_M_F.setDirection(DcMotor.Direction.FORWARD);
    }
    protected void DirectionFORWARD(){
        R_M_B.setDirection(DcMotor.Direction.FORWARD);
        R_M_F.setDirection(DcMotor.Direction.FORWARD);
        L_M_B.setDirection(DcMotor.Direction.REVERSE);
        L_M_F.setDirection(DcMotor.Direction.REVERSE);
    }
    protected void WhileBusyTelemetr(){
        while(R_M_B.isBusy()&&R_M_F.isBusy()&&L_M_F.isBusy()&&L_M_B.isBusy()){
            telemetry.addLine("Encoders working");
            telemetry.addData("R_M_F: ",R_M_F.getCurrentPosition()/1120);
            telemetry.addData("R_M_B: ",R_M_B.getCurrentPosition()/1120);
            telemetry.addData("L_M_F: ",L_M_F.getCurrentPosition()/1120);
            telemetry.addData("L_M_B: ",L_M_B.getCurrentPosition()/1120);
            telemetry.update();
        }
    }
    protected void ResetEncoders(){
        R_M_F.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R_M_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L_M_F.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L_M_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    protected void SetPosition(int Target){
        R_M_F.setTargetPosition(Target);
        R_M_B.setTargetPosition(Target);
        L_M_F.setTargetPosition(Target);
        L_M_B.setTargetPosition(Target);
    }
    protected void RunToPosition(){
        R_M_F.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        R_M_B.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        L_M_F.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        L_M_B.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void Reverse(double power){
        Forward(-power);
    }
    private void StopDriving(){
        Forward(0);
    }
}
