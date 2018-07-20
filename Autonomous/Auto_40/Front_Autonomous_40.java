package org.firstinspires.ftc.teamcode.RelicRecovery.Autonomous.Auto_40;

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Jorge on 23/11/2017.
 */
//@Autonomous(name="Front_Autonomous_40")
public class Front_Autonomous_40 extends LinearOpMode {
    //WHEELS
    protected DcMotor L_M_F = null;
    protected DcMotor L_M_B = null;
    protected DcMotor R_M_F = null;
    protected DcMotor R_M_B = null;
    protected DcMotor Lifter = null;

    //SERVOS
    protected Servo R_S_J = null;
    protected Servo L_S_J = null;
    protected Servo R_S_C = null;
    protected Servo L_S_C = null;

    //SENSORS
    protected ColorSensor LineSensor;
    protected ColorSensor JewelSensor;
    protected ModernRoboticsI2cGyro GyroSensor;
    protected ModernRoboticsI2cGyro GyroS;
    protected ModernRoboticsI2cRangeSensor RangeSensor;

    //VUFORIA
    protected VuforiaLocalizer vuforia;

    //VARIABLES
    protected char Front;
    protected String Multi;
    protected String JewelColor;
    protected String StoneColor;
    protected String blue = "blue";
    protected String red = "red";

    @Override
    public void runOpMode() throws InterruptedException {
        //HARDWARE MAPPING
        L_M_F = hardwareMap.dcMotor.get("L_M_F");
        L_M_B = hardwareMap.dcMotor.get("L_M_B");
        R_M_F = hardwareMap.dcMotor.get("R_M_F");
        R_M_B = hardwareMap.dcMotor.get("R_M_B");
        Lifter = hardwareMap.dcMotor.get("Lifter");

        R_M_F.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        R_M_B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        L_M_F.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        L_M_B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        L_M_F.setDirection(DcMotor.Direction.REVERSE);
        L_M_B.setDirection(DcMotor.Direction.REVERSE);

        R_S_J = hardwareMap.servo.get("R_S_J");
        L_S_J = hardwareMap.servo.get("L_S_J");
        R_S_C = hardwareMap.servo.get("R_S_C");
        L_S_C = hardwareMap.servo.get("L_S_C");

        JewelSensor = hardwareMap.colorSensor.get("JewelSensor");
        //ColorSensor.setI2cAddress(I2cAddr.create7bit(0x26));//4c 0x26
        LineSensor = hardwareMap.colorSensor.get("LineSensor");
        //ColorSensor2.setI2cAddress(I2cAddr.create7bit(0x1e));//3c
        GyroSensor = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("GyroSensor");
        GyroS = (ModernRoboticsI2cGyro) GyroSensor;
        RangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "RangeSensor");
        LineSensor.enableLed(true);
        JewelSensor.enableLed(true);

        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        GyroSensor.calibrate();

        while (!isStopRequested() && GyroSensor.isCalibrating()) {
            sleep(500);
            idle();
        }

        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();

        R_S_J.setPosition(.6);
        L_S_J.setPosition(1);

       waitForStart();
        GrabCubeandLift(.7, .7, .5, 600);
        CheckStoneColor();
        sleep(250);
        HitJewel(.3, 0, .8, .6, 1);
        sleep(250);
        ForwardByRotations(.1,.5);
        sleep(1000);
        Lifter.setPower(1);
        sleep(1000);
        Lifter.setPower(0);
        Lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FirstTurn();
        StopDriving();
        ForwardByRotations(.1,1);
        SecondTurn();
        ReverseByTime(.3,450);
        ThrowGlyph(.8,.6, 1, 1000);
        ForwardByDistance(.3, 15);
        HitGlyph();
        R_S_C.setPosition(1);
        ReverseByTime(.2,150);
        telemetry.addLine("AUTONOMOUS FINISHED");
        telemetry.update();
    }

    //FUNCTIONS
    protected void Forward(double power) {
        R_M_F.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        R_M_B.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        L_M_F.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        L_M_B.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        L_M_F.setDirection(DcMotor.Direction.REVERSE);
        L_M_B.setDirection(DcMotor.Direction.REVERSE);
        R_M_B.setDirection(DcMotor.Direction.FORWARD);
        R_M_F.setDirection(DcMotor.Direction.FORWARD);

        R_M_F.setPower(power);
        R_M_B.setPower(power);
        L_M_F.setPower(power);
        L_M_B.setPower(power);
    }

    protected void Reverse(double power) {
        R_M_F.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        R_M_B.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        L_M_F.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        L_M_B.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        L_M_F.setDirection(DcMotor.Direction.REVERSE);
        L_M_B.setDirection(DcMotor.Direction.REVERSE);
        R_M_B.setDirection(DcMotor.Direction.FORWARD);
        R_M_F.setDirection(DcMotor.Direction.FORWARD);

        R_M_F.setPower(-power);
        R_M_B.setPower(-power);
        L_M_F.setPower(-power);
        L_M_B.setPower(-power);
    }

    protected void LeftWheelsTurn(double power) {
        R_M_F.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        R_M_B.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        L_M_F.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        L_M_B.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        L_M_F.setDirection(DcMotor.Direction.REVERSE);
        L_M_B.setDirection(DcMotor.Direction.REVERSE);
        R_M_B.setDirection(DcMotor.Direction.FORWARD);
        R_M_F.setDirection(DcMotor.Direction.FORWARD);

        L_M_F.setPower(power);
        L_M_B.setPower(power);
    }

    protected void RightWheelsTurn(double power) {
        R_M_F.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        R_M_B.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        L_M_F.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        L_M_B.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        L_M_F.setDirection(DcMotor.Direction.REVERSE);
        L_M_B.setDirection(DcMotor.Direction.REVERSE);
        R_M_B.setDirection(DcMotor.Direction.FORWARD);
        R_M_F.setDirection(DcMotor.Direction.FORWARD);

        R_M_F.setPower(power);
        R_M_B.setPower(power);
    }

    protected void LeftTurn(double power) {
        R_M_F.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        R_M_B.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        L_M_F.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        L_M_B.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        L_M_F.setDirection(DcMotor.Direction.REVERSE);
        L_M_B.setDirection(DcMotor.Direction.REVERSE);
        R_M_B.setDirection(DcMotor.Direction.FORWARD);
        R_M_F.setDirection(DcMotor.Direction.FORWARD);

        R_M_F.setPower(power);
        R_M_B.setPower(power);
        L_M_F.setPower(-power);
        L_M_B.setPower(-power);
    }

    protected void RightTurn(double power) {
        R_M_F.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        R_M_B.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        L_M_F.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        L_M_B.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        L_M_F.setDirection(DcMotor.Direction.REVERSE);
        L_M_B.setDirection(DcMotor.Direction.REVERSE);
        R_M_B.setDirection(DcMotor.Direction.FORWARD);
        R_M_F.setDirection(DcMotor.Direction.FORWARD);

        R_M_F.setPower(-power);
        R_M_B.setPower(-power);
        L_M_F.setPower(power);
        L_M_B.setPower(power);
    }

    protected void StopDriving() {
        R_M_F.setPower(0);
        R_M_B.setPower(0);
        L_M_F.setPower(0);
        L_M_B.setPower(0);
    }

    protected void ForwardByTime(double power, int time) {
        R_M_F.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        R_M_B.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        L_M_F.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        L_M_B.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Forward(power);
        sleep(time);
        StopDriving();
        R_M_F.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        R_M_B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        L_M_F.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        L_M_B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    protected void ReverseByTime(double power, int time) {
        R_M_F.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        R_M_B.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        L_M_F.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        L_M_B.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Forward(-power);
        sleep(time);
        StopDriving();
        R_M_F.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        R_M_B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        L_M_F.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        L_M_B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    protected void LeftTurnByTime(double power, int time) {
        R_M_F.setPower(power);
        R_M_B.setPower(power);
        L_M_F.setPower(-power);
        L_M_B.setPower(-power);
        sleep(time);
        StopDriving();
    }

    protected void RightTurnByTime(double power, int time) {
        R_M_F.setPower(-power);
        R_M_B.setPower(-power);
        L_M_F.setPower(power);
        L_M_B.setPower(power);
        sleep(time);
        StopDriving();
    }

    protected void ForwardByRotations(double power, double rotations) {
        R_M_F.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R_M_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L_M_F.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L_M_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R_M_F.setDirection(DcMotor.Direction.FORWARD);
        R_M_B.setDirection(DcMotor.Direction.FORWARD);
        L_M_F.setDirection(DcMotor.Direction.REVERSE);
        L_M_B.setDirection(DcMotor.Direction.REVERSE);

        double Target = (rotations * 1120);
        telemetry.addData("> Target: ", Target);
        telemetry.update();
        sleep(1000);
        L_M_F.setPower(power);
        R_M_F.setPower(power);
        R_M_B.setPower(power);
        L_M_B.setPower(power);
        while (Math.abs(R_M_B.getCurrentPosition()) < Target && Math.abs(R_M_F.getCurrentPosition()) < Target && Math.abs(L_M_F.getCurrentPosition()) < Target && Math.abs(L_M_B.getCurrentPosition()) < Target) {
            R_M_B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            R_M_F.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            L_M_F.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            L_M_B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addLine("Encoders working");
            telemetry.addData("> Target: ", Target);
            telemetry.addData("> Rotation: ", R_M_F.getCurrentPosition() / 1120);
            telemetry.addData("> Math Position: ", Math.abs(R_M_F.getCurrentPosition()));
            telemetry.addData("> Rotation: ", R_M_B.getCurrentPosition() / 1120);
            telemetry.addData("> Math Position: ", Math.abs(R_M_B.getCurrentPosition()));
            telemetry.addData("> Rotation: ", L_M_F.getCurrentPosition() / 1120);
            telemetry.addData("> Math Position: ", Math.abs(L_M_F.getCurrentPosition()));
            telemetry.addData("> Rotation: ", L_M_B.getCurrentPosition() / 1120);
            telemetry.addData("> Math Position: ", Math.abs(L_M_B.getCurrentPosition()));
            telemetry.update();
        }
        R_M_F.setPower(0);
        L_M_F.setPower(0);
        R_M_B.setPower(0);
        L_M_B.setPower(0);
        //StopDriving();
      /*  R_M_F.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        L_M_F.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        L_M_B.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        R_M_B.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/
    }

    protected void ReverseByRotations(double power, double rotations) {
        sleep(500);
        R_M_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R_M_B.setDirection(DcMotor.Direction.REVERSE);
        R_M_F.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R_M_F.setDirection(DcMotor.Direction.REVERSE);
        L_M_F.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L_M_F.setDirection(DcMotor.Direction.FORWARD);
        L_M_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L_M_B.setDirection(DcMotor.Direction.FORWARD);
        sleep(500);

        double Target = (rotations * 1120 * -1);
        telemetry.addData("> Target: ", Target);
        telemetry.update();
        sleep(1000);
        R_M_B.setPower(power);
        L_M_B.setPower(power);
        R_M_F.setPower(power);
        L_M_F.setPower(power);
        while (R_M_B.getCurrentPosition() > Target && R_M_F.getCurrentPosition() > Target && L_M_B.getCurrentPosition() > Target && L_M_F.getCurrentPosition() > Target) {
            R_M_B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            R_M_F.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            L_M_F.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            L_M_B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addLine("Encoders working");
            telemetry.addData("> Target: ", Target);
            telemetry.addData("> Rotation: ", R_M_F.getCurrentPosition() / 1120);
            telemetry.addData("> Math Position: ", Math.abs(R_M_F.getCurrentPosition()));
            telemetry.addData("> Rotation: ", R_M_B.getCurrentPosition() / 1120);
            telemetry.addData("> Math Position: ", Math.abs(R_M_B.getCurrentPosition()));
            telemetry.addData("> Rotation: ", L_M_F.getCurrentPosition() / 1120);
            telemetry.addData("> Math Position: ", Math.abs(L_M_F.getCurrentPosition()));
            telemetry.addData("> Rotation: ", L_M_B.getCurrentPosition() / 1120);
            telemetry.addData("> Math Position: ", Math.abs(L_M_B.getCurrentPosition()));
            telemetry.update();
        }
        R_M_B.setPower(0);
        L_M_B.setPower(0);
        R_M_F.setPower(0);
        L_M_F.setPower(0);
        // StopDriving();
       /* R_M_F.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        L_M_F.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        R_M_B.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        L_M_B.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/
    }

    protected void LeftTurnByRotations(double power, int rotations) {
        R_M_F.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R_M_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L_M_F.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L_M_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        R_M_F.setTargetPosition(rotations * 1120);
        R_M_B.setTargetPosition(rotations * 1120);
        L_M_F.setTargetPosition(-(rotations * 1120));
        L_M_B.setTargetPosition(-(rotations * 1120));

        R_M_F.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        R_M_B.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        L_M_F.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        L_M_B.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LeftTurn(power);
        while (R_M_F.isBusy() && R_M_B.isBusy() && L_M_F.isBusy() && L_M_B.isBusy()) {
            telemetry.addLine("Encoders working");
            telemetry.addData("R_M_F Rotation: ", R_M_F.getCurrentPosition() / 1120);
            telemetry.addData("R_M_B Rotation: ", R_M_B.getCurrentPosition() / 1120);
            telemetry.addData("L_M_F Rotation: ", L_M_F.getCurrentPosition() / 1120);
            telemetry.addData("L_M_B Rotation: ", L_M_B.getCurrentPosition() / 1120);
            telemetry.update();
        }
        StopDriving();
    }

    protected void RightTurnByRotations(double power, int rotations) {
        R_M_F.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R_M_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L_M_F.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L_M_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        R_M_F.setTargetPosition(-(rotations * 1120));
        R_M_B.setTargetPosition(-(rotations * 1120));
        L_M_F.setTargetPosition(rotations * 1120);
        L_M_B.setTargetPosition(rotations * 1120);

        R_M_F.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        R_M_B.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        L_M_F.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        L_M_B.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        RightTurn(power);
        while (R_M_F.isBusy() && R_M_B.isBusy() && L_M_F.isBusy() && L_M_B.isBusy()) {
            telemetry.addLine("Encoders working");
            telemetry.addData("R_M_F Rotation: ", R_M_F.getCurrentPosition() / 1120);
            telemetry.addData("R_M_B Rotation: ", R_M_B.getCurrentPosition() / 1120);
            telemetry.addData("L_M_F Rotation: ", L_M_F.getCurrentPosition() / 1120);
            telemetry.addData("L_M_B Rotation: ", L_M_B.getCurrentPosition() / 1120);
            telemetry.update();
        }
        StopDriving();
    }

    protected void RightWheelsTurnByRotations(double power, int rotations) {
        R_M_F.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R_M_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L_M_F.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L_M_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        R_M_F.setTargetPosition(-(rotations * 1120));
        R_M_B.setTargetPosition(-(rotations * 1120));

        R_M_F.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        R_M_B.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        RightWheelsTurn(power);
        while (R_M_F.isBusy() && R_M_B.isBusy()) {
            telemetry.addLine("Encoders working");
            telemetry.addData("R_M_F Rotation: ", R_M_F.getCurrentPosition() / 1120);
            telemetry.addData("R_M_B Rotation: ", R_M_B.getCurrentPosition() / 1120);
            telemetry.update();
        }
        StopDriving();
    }

    protected void LeftWheelsTurnByRotations(double power, int rotations) {
        R_M_F.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R_M_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L_M_F.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L_M_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        L_M_F.setTargetPosition(-(rotations * 1120));
        L_M_B.setTargetPosition(-(rotations * 1120));

        L_M_F.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        L_M_B.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        RightWheelsTurn(power);
        while (L_M_F.isBusy() && L_M_B.isBusy()) {
            telemetry.addLine("Encoders working");
            telemetry.addData("L_M_F Rotation: ", L_M_F.getCurrentPosition() / 1120);
            telemetry.addData("L_M_B Rotation: ", L_M_B.getCurrentPosition() / 1120);
            telemetry.update();
        }
        StopDriving();
    }

    protected void LeftHorizontalByRotations(double power, int rotations) {
        sleep(500);
        R_M_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R_M_B.setDirection(DcMotor.Direction.FORWARD);
        R_M_F.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R_M_F.setDirection(DcMotor.Direction.REVERSE);
        L_M_F.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L_M_F.setDirection(DcMotor.Direction.FORWARD);
        L_M_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L_M_B.setDirection(DcMotor.Direction.REVERSE);
        sleep(500);

        int Target = (rotations * 1120);
        telemetry.addData("> Target: ", Target);
        telemetry.update();
        sleep(1000);
        L_M_B.setPower(.1);
        L_M_F.setPower(.1);
        R_M_B.setPower(.1);
        R_M_F.setPower(.1);
        while (R_M_B.getCurrentPosition() > Target && R_M_F.getCurrentPosition() > Target && L_M_B.getCurrentPosition() > Target && L_M_F.getCurrentPosition() > Target) {
            R_M_B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            R_M_F.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            L_M_F.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            L_M_B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addLine("Encoders working");
            telemetry.addData("> Target: ", Target);
            telemetry.addData("> Rotation: ", R_M_F.getCurrentPosition() / 1120);
            telemetry.addData("> Math Position: ", Math.abs(R_M_F.getCurrentPosition()));
            telemetry.addData("> Rotation: ", R_M_B.getCurrentPosition() / 1120);
            telemetry.addData("> Math Position: ", Math.abs(R_M_B.getCurrentPosition()));
            telemetry.addData("> Rotation: ", L_M_F.getCurrentPosition() / 1120);
            telemetry.addData("> Math Position: ", Math.abs(L_M_F.getCurrentPosition()));
            telemetry.addData("> Rotation: ", L_M_B.getCurrentPosition() / 1120);
            telemetry.addData("> Math Position: ", Math.abs(L_M_B.getCurrentPosition()));
            telemetry.update();
        }
        L_M_B.setPower(0);
        L_M_F.setPower(0);
        R_M_F.setPower(0);
        R_M_B.setPower(0);
    }

    protected void CheckJewelColor() {
        float hsvValues[] = {0, 0, 0};

        Color.RGBToHSV(JewelSensor.red() * 8, JewelSensor.green() * 8, JewelSensor.blue() * 8, hsvValues);
        if (JewelSensor.blue() > JewelSensor.red() && JewelSensor.blue() > JewelSensor.green()) {
            telemetry.addLine("Color: blue");
            telemetry.update();
            JewelColor = blue;
        } else if (JewelSensor.red() > JewelSensor.blue() && JewelSensor.red() > JewelSensor.green()) {
            telemetry.addLine("Color: red");
            telemetry.update();
            JewelColor = red;
        } else {
            telemetry.addLine(">  NO DETECTED");
        }
        telemetry.addData(">  Jewel Color: ", JewelColor);
        telemetry.addLine(">  CheckJewelColor phase complete");
        telemetry.update();
    }

    protected void CheckStoneColor() {
        float hsvValues[] = {0, 0, 0};

        Color.RGBToHSV(LineSensor.red() * 8, LineSensor.green() * 8, LineSensor.blue() * 8, hsvValues);
        if (LineSensor.blue() > LineSensor.red() && LineSensor.blue() > LineSensor.green()) {
            telemetry.addLine("Color: blue");
            telemetry.update();
            StoneColor = blue;
        } else if (LineSensor.red() > LineSensor.blue() && LineSensor.red() > LineSensor.green()) {
            telemetry.addLine("Color: red");
            telemetry.update();
            StoneColor = red;
        } else {
            telemetry.addLine(">  NO STONE COLOR DETECTED");
        }
        telemetry.addData(">  Stone Color: ", StoneColor);
        telemetry.addLine(">  CheckJewelColor phase complete");
        telemetry.update();
    }

    protected void CheckGyroValues() {
        telemetry.addData("0", "Heading %03d", GyroSensor.getHeading());
        telemetry.addData("1", "Int. Ang. %03d", GyroSensor.getIntegratedZValue());
        telemetry.addData("2", "X av. %03d", GyroSensor.rawX());
        telemetry.addData("3", "Y av. %03d", GyroSensor.rawY());
        telemetry.addData("4", "Z av. %03d", GyroSensor.rawZ());
        telemetry.update();
    }

    protected void CheckRangeDistanceCM() {
        telemetry.addData("cm optical", "%.2f cm", RangeSensor.cmOptical());
        telemetry.addData("cm", "%.2f cm", RangeSensor.getDistance(DistanceUnit.CM));
        telemetry.update();
    }

    protected void GyroHeading() {
        telemetry.addData("Heading: ", GyroSensor.getHeading());
        telemetry.update();
    }

    protected void ForwardByDistance(double power, double CMTargeted) {
        telemetry.addData("Distance: ", RangeSensor.cmUltrasonic());
        telemetry.update();
        while (RangeSensor.cmUltrasonic() > CMTargeted) {
            Forward(power);
            telemetry.addData("Distance: ", RangeSensor.cmUltrasonic());
            telemetry.update();
        }
        StopDriving();
    }

    protected void VuMarkCheck(){
        switch (Multi) {
            case "LEFT":
                telemetry.addLine("Izquierda");
                telemetry.update();
                break;
            case "RIGHT":
                telemetry.addLine("Derecha");
                telemetry.update();
                break;
            case "CENTER":
                telemetry.addLine("Centro");
                telemetry.update();
                break;
            default:
                telemetry.addLine("ERROR - VuMarkCheck");
                telemetry.update();
                break;
        }
    }

    protected void GrabCubeandLift(double posR, double posL, double power, int time){
        R_S_C.setPosition(posR);
        L_S_C.setPosition(posL);
        sleep(850);
        Lifter.setPower(power);
        sleep(time);
        Lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lifter.setPower(0);
        Lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addLine(">  GrabCubeandLift phase complete");
        telemetry.update();
    }
    protected void HitJewel(double power, double R_S_Jpos1,double R_S_Jpos2,double L_S_Jpos1,double L_S_Jpos2) {
        int wait = 400;
        if (StoneColor == red) {
            R_S_J.setPosition(R_S_Jpos1);
            sleep(3000);
            CheckJewelColor();
            sleep(2000);
            if (JewelColor == "red") {
                RightWheelsTurn(-power);
                sleep(wait);
                StopDriving();
                R_S_J.setPosition(R_S_Jpos2);
                Front='N';
                sleep(wait);
                RightWheelsTurn(power);
                sleep(wait/2);
                StopDriving();
            } else if (JewelColor == "blue") {
                RightWheelsTurn(power);
                sleep(wait);
                StopDriving();
                R_S_J.setPosition(R_S_Jpos2);
                sleep(wait);
                Front='Y';
                RightWheelsTurn(-power);
                sleep(wait/2);
                StopDriving();
            } else {
                telemetry.addLine("ERROR - JEWEL COLOR");
                telemetry.update();
            }
        } else if (StoneColor == blue) {
            L_S_J.setPosition(L_S_Jpos1);
            sleep(3000);
            CheckJewelColor();
            sleep(2000);
            if (JewelColor == "red") {
                LeftWheelsTurn(power);
                sleep(wait);
                StopDriving();
                L_S_J.setPosition(L_S_Jpos2);
                Front='Y';
                LeftWheelsTurn(-power);
                sleep(wait/2);
                StopDriving();
            } else if (JewelColor == "blue") {
                LeftWheelsTurn(-power);
                sleep(wait);
                StopDriving();
                L_S_J.setPosition(L_S_Jpos2);
                sleep(wait);
                Front='N';
                LeftWheelsTurn(power);
                sleep(wait/2);
                StopDriving();
            } else {
                telemetry.addLine("ERROR - JEWEL COLOR");
                telemetry.update();
                R_S_J.setPosition(R_S_Jpos2);
                L_S_J.setPosition(L_S_Jpos2);
            }
        } else {
            telemetry.addLine("ERROR - STONE COLOR");
            telemetry.update();
            R_S_J.setPosition(R_S_Jpos2);
            L_S_J.setPosition(L_S_Jpos2);
        }
    }
    protected void WayToCryptobox(){
        if(Front=='Y'&&StoneColor==red){
            telemetry.addLine("Front & Red");
            telemetry.update();
            LeftWheelsTurnByRotations(.5,1);
            ForwardByRotations(.5,3);
        }
        else if(Front=='Y'&&StoneColor==blue) {
            telemetry.addLine("Front & Blue");
            telemetry.update();
            RightWheelsTurnByRotations(.5,1);
            ForwardByRotations(.5,3);
        }
        else if (Front=='N'&&StoneColor==red){
            telemetry.addLine("Back & Red");
            telemetry.update();
            ForwardByRotations(1,6);
            RightTurnByRotations(.5,2);
        }
        else if (Front=='N'&&StoneColor==blue){
            telemetry.addLine("Back & Blue");
            telemetry.update();
            ForwardByRotations(1,6);
            LeftTurnByRotations(.5,2);
        }
        else{
            telemetry.addLine("ERROR - WAY TO CRYPTOBOX");
            telemetry.update();
        }
    }
    protected void FirstTurn () {

        if (StoneColor == red) {
            L_M_F.setDirection(DcMotor.Direction.REVERSE);
            L_M_B.setDirection(DcMotor.Direction.REVERSE);
            R_M_B.setDirection(DcMotor.Direction.FORWARD);
            R_M_F.setDirection(DcMotor.Direction.FORWARD);

            R_M_F.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            R_M_B.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            L_M_F.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            L_M_B.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            int angulo = -90;
            double power = .3;
            double zTotal;
            zTotal = GyroSensor.getIntegratedZValue();
            while (Math.abs(zTotal - angulo) > 3) {
                if (zTotal > angulo) {
                    L_M_B.setPower(-power);
                    R_M_F.setPower(power);
                    R_M_B.setPower(power);
                    L_M_F.setPower(-power);
                } else if (zTotal < angulo) {
                    L_M_B.setPower(power);
                    R_M_F.setPower(-power);
                    R_M_B.setPower(-power);
                    L_M_F.setPower(power);
                }
                zTotal = GyroSensor.getIntegratedZValue();
                telemetry.addData("Grados: ", zTotal);
                telemetry.update();
            }
            StopDriving();
            zTotal = GyroSensor.getIntegratedZValue();
            telemetry.addData("Grados: ", zTotal);
            telemetry.update();
            R_M_F.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            R_M_B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            L_M_F.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            L_M_B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else if(StoneColor == blue){
            L_M_F.setDirection(DcMotor.Direction.FORWARD);
            L_M_B.setDirection(DcMotor.Direction.FORWARD);
            R_M_B.setDirection(DcMotor.Direction.REVERSE);
            R_M_F.setDirection(DcMotor.Direction.REVERSE);

            R_M_F.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            R_M_B.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            L_M_F.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            L_M_B.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            int angulo = 90;
            double power = .3;
            double zTotal;
            zTotal = GyroSensor.getIntegratedZValue();
            while (Math.abs(zTotal - angulo) > 3) {
                if (zTotal > angulo) {
                    L_M_B.setPower(power);
                    R_M_F.setPower(-power);
                    R_M_B.setPower(-power);
                    L_M_F.setPower(power);
                } else if (zTotal < angulo) {
                    L_M_B.setPower(-power);
                    R_M_F.setPower(power);
                    R_M_B.setPower(power);
                    L_M_F.setPower(-power);
                }
                zTotal = GyroSensor.getIntegratedZValue();
                telemetry.addData("Grados: ", zTotal);
                telemetry.update();
            }
            StopDriving();
            zTotal = GyroSensor.getIntegratedZValue();
            telemetry.addData("Grados: ", zTotal);
            telemetry.update();
            R_M_F.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            R_M_B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            L_M_F.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            L_M_B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    protected void ThrowGlyph(double posR, double posL, double power, int time){
        Lifter.setPower(-power);
        sleep(time);
        Lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lifter.setPower(0);
        Lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        R_S_C.setPosition(posR);
        L_S_C.setPosition(posL);
        telemetry.addLine(">  GrabCubeandLift phase complete");
        telemetry.update();
    }
    protected void HitGlyph (){
        LeftTurnByTime(.5,500);
    }
    protected void SecondTurn (){
            L_M_F.setDirection(DcMotor.Direction.REVERSE);
            L_M_B.setDirection(DcMotor.Direction.REVERSE);
            R_M_B.setDirection(DcMotor.Direction.FORWARD);
            R_M_F.setDirection(DcMotor.Direction.FORWARD);

            R_M_F.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            R_M_B.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            L_M_F.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            L_M_B.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            int angulo = 0;
            double power = .3;
            double zTotal;
            zTotal = GyroSensor.getIntegratedZValue();
            while (Math.abs(zTotal - angulo) > 3) {
                if (zTotal > angulo) {
                    L_M_B.setPower(-power);
                    R_M_F.setPower(power);
                    R_M_B.setPower(power);
                    L_M_F.setPower(-power);
                } else if (zTotal < angulo) {
                    L_M_B.setPower(power);
                    R_M_F.setPower(-power);
                    R_M_B.setPower(-power);
                    L_M_F.setPower(power);
                }
                zTotal = GyroSensor.getIntegratedZValue();
                telemetry.addData("Grados: ", zTotal);
                telemetry.update();
            }
            StopDriving();
            zTotal = GyroSensor.getIntegratedZValue();
            telemetry.addData("Grados: ", zTotal);
            telemetry.update();
            R_M_F.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            R_M_B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            L_M_F.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            L_M_B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
