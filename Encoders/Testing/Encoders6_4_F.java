package org.firstinspires.ftc.teamcode.RelicRecovery.Encoders.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Jorge on 01/12/2017.
 */
//@TeleOp(name ="Encoders6_4_F",group = "TeleOp")
public class Encoders6_4_F extends LinearOpMode {
    private DcMotor R_M_F = null;
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
        L_M_B = hardwareMap.dcMotor.get("L_M_B");

        L_M_B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //ANDYMARK TICKS PER REVOLUTIONS 1120

        waitForStart();
        telemetry.addLine("Starting encoders");
        telemetry.update();
        ForwardDistance(.1,5);
        ReverseDistance(.1,5);
        telemetry.addLine("Encoders Finished");
        telemetry.update();
    }

    protected void ForwardDistance(double power, int rotations){
        L_M_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L_M_B.setDirection(DcMotor.Direction.REVERSE);

        int Target = (rotations*1120);
        telemetry.addData("> Target: ",Target);
        telemetry.update();
        sleep(1000);
        L_M_B.setPower(power);
        while(Math.abs(L_M_B.getCurrentPosition())<Target){
            L_M_B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addLine("Encoders working");
            telemetry.addData("> Target: ",Target);
            telemetry.addData("> Rotation: ",L_M_B.getCurrentPosition()/1120);
            telemetry.addData("> Math Position: ",Math.abs(L_M_B.getCurrentPosition()));
            telemetry.update();
        }
        L_M_B.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    protected void ReverseDistance(double power, int rotations){
        sleep(500);
        L_M_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L_M_B.setDirection(DcMotor.Direction.FORWARD);
        sleep(500);

        int Target = (rotations*1120*-1);
        telemetry.addData("> Target: ",Target);
        telemetry.update();
        sleep(1000);
        L_M_B.setPower(power);
        while((L_M_B.getCurrentPosition())>Target){
            L_M_B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addLine("Encoders working");
            telemetry.addData("> Target: ",Target);
            telemetry.addData("> Rotation: ",L_M_B.getCurrentPosition()/1120);
            telemetry.addData("> Math Position: ",Math.abs(L_M_B.getCurrentPosition()));
            telemetry.update();
        }
        L_M_B.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    private void Forward(double power){
        L_M_B.setPower(power);
        L_M_B.setPower(power);
        L_M_B.setPower(power);
        L_M_B.setPower(power);
    }
    protected void DirectionREVERSE(){
        L_M_B.setDirection(DcMotor.Direction.REVERSE);
        L_M_B.setDirection(DcMotor.Direction.REVERSE);
        L_M_B.setDirection(DcMotor.Direction.FORWARD);
        L_M_B.setDirection(DcMotor.Direction.FORWARD);
    }
    protected void DirectionFORWARD(){
        L_M_B.setDirection(DcMotor.Direction.FORWARD);
        L_M_B.setDirection(DcMotor.Direction.FORWARD);
        L_M_B.setDirection(DcMotor.Direction.REVERSE);
        L_M_B.setDirection(DcMotor.Direction.REVERSE);
    }
    protected void WhileBusyTelemetr(){
        while(L_M_B.isBusy()&&L_M_B.isBusy()&&L_M_B.isBusy()&&L_M_B.isBusy()){
            telemetry.addLine("Encoders working");
            telemetry.addData("L_M_B: ",L_M_B.getCurrentPosition()/1120);
            telemetry.addData("L_M_B: ",L_M_B.getCurrentPosition()/1120);
            telemetry.addData("L_M_B: ",L_M_B.getCurrentPosition()/1120);
            telemetry.addData("L_M_B: ",L_M_B.getCurrentPosition()/1120);
            telemetry.update();
        }
    }
    protected void ResetEncoders(){
        L_M_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L_M_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L_M_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L_M_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    protected void SetPosition(int Target){
        L_M_B.setTargetPosition(Target);
        L_M_B.setTargetPosition(Target);
        L_M_B.setTargetPosition(Target);
        L_M_B.setTargetPosition(Target);
    }
    protected void RunToPosition(){
        L_M_B.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        L_M_B.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        L_M_B.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        L_M_B.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void Reverse(double power){
        Forward(-power);
    }
    private void StopDriving(){
        Forward(0);
    }
}
