package org.firstinspires.ftc.teamcode.RelicRecovery.Sensors.Practicals;

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
@Autonomous(name="GyroPractical")
public class GyroPractical extends LinearOpMode {
    protected DcMotor L_M_F = null;
    protected DcMotor L_M_B = null;
    protected DcMotor R_M_F = null;
    protected DcMotor R_M_B = null;
    protected DcMotor Lifter = null;
    public DcMotor GlyphGrabberR = null;
    public DcMotor GlyphGrabberL = null;

    protected Servo R_S_J = null;
    protected Servo L_S_J = null;
    public Servo GlyphServoGrabber;

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

        L_M_F = hardwareMap.dcMotor.get("L_M_F");
        L_M_B = hardwareMap.dcMotor.get("L_M_B");
        R_M_F = hardwareMap.dcMotor.get("R_M_F");
        R_M_B = hardwareMap.dcMotor.get("R_M_B");
        Lifter = hardwareMap.dcMotor.get("Lifter");
        GlyphGrabberL = hardwareMap.dcMotor.get("GlyphGrabberL");
        GlyphGrabberR = hardwareMap.dcMotor.get("GlyphGrabberR");

        R_S_J = hardwareMap.servo.get("R_S_J");
        L_S_J = hardwareMap.servo.get("L_S_J");
        GlyphServoGrabber = hardwareMap.servo.get("GlyphServoGrabber");

        R_M_F.setDirection(DcMotor.Direction.FORWARD);
        R_M_B.setDirection(DcMotor.Direction.FORWARD);
        L_M_F.setDirection(DcMotor.Direction.REVERSE);
        L_M_B.setDirection(DcMotor.Direction.REVERSE);
        Lifter.setDirection(DcMotor.Direction.REVERSE);
        GlyphGrabberL.setDirection(DcMotor.Direction.REVERSE);

        JewelSensor = hardwareMap.colorSensor.get("JewelSensor");
        LineSensor = hardwareMap.colorSensor.get("LineSensor");
        GyroSensor = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("GyroSensor");
        GyroS = (ModernRoboticsI2cGyro) GyroSensor;
        RangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "RangeSensor");
        LineSensor.enableLed(true);
        JewelSensor.enableLed(true);

        RunWithoutEncoders();

        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        GyroSensor.calibrate();

        while (!isStopRequested() && GyroSensor.isCalibrating()) {
            sleep(500);
            idle();
        }

        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();

        R_S_J.setPosition(.1);
        L_S_J.setPosition(1);
        GlyphServoGrabber.setPosition(.5);

        waitForStart();
        double zTotal;
        while (opModeIsActive()){
            zTotal = GyroSensor.getIntegratedZValue();
            telemetry.addData("Grados: ", zTotal);
            telemetry.update();
        }
    }

    public void Forward(double power){
        L_M_F.setPower(power);
        R_M_F.setPower(power);
        L_M_B.setPower(power);
        R_M_B.setPower(power);
    }
    public void Reverse(double power){
        Forward(-power);
    }
    public void RightTurn(double power){
        R_M_B.setPower(-power);
        L_M_F.setPower(power);
        L_M_B.setPower(power);
        R_M_F.setPower(-power);
    }
    public void LeftTurn(double power){
        L_M_B.setPower(-power);
        R_M_F.setPower(power);
        R_M_B.setPower(power);
        L_M_F.setPower(-power);
    }
    public void RightWheelsTurn(double power){
        R_M_F.setPower(power);
        R_M_B.setPower(power);
    }
    public void LeftWheelsTurn(double power){
        L_M_F.setPower(power);
        L_M_B.setPower(power);
    }
    public void StopDriving(){
        Forward(0);
    }

    public void ForwardByTime(double power, int time){
        Forward(power);
        sleep(time);
        StopDriving();
    }
    public void ReverseByTime(double power, int time){
        Reverse(power);
        sleep(time);
        StopDriving();
    }
    public void RightTurnByTime(double power, int time){
        R_M_B.setPower(-power);
        L_M_F.setPower(power);
        L_M_B.setPower(power);
        R_M_F.setPower(-power);
        sleep(time);
        StopDriving();
    }
    public void LeftTurnByTime(double power, int time){
        L_M_B.setPower(-power);
        R_M_F.setPower(power);
        R_M_B.setPower(power);
        L_M_F.setPower(-power);
        sleep(time);
        StopDriving();
    }
    public void RightWheelsTurnByTime(double power, int time) {
        R_M_F.setPower(power);
        R_M_B.setPower(power);
        sleep(time);
        StopDriving();
    }
    public void LeftWheelsTurnByTime(double power, int time){
        L_M_F.setPower(power);
        L_M_B.setPower(power);
        sleep(time);
        StopDriving();
    }

    public void ForwardStraight(double power){
        L_M_F.setPower(power);
        R_M_F.setPower(power);
        L_M_B.setPower(power);
        R_M_B.setPower(power+.2);//+.2
    }
    public void ReverseStraight(double power){
        L_M_F.setPower(-power-.1);
        R_M_F.setPower(-power);
        L_M_B.setPower(-power-.1);
        R_M_B.setPower(-power);
    }

    public void ForwardStraightByTime(double power, int time){
        L_M_F.setPower(power+.1);
        R_M_F.setPower(power);
        L_M_B.setPower(power+.1);
        R_M_B.setPower(power+.2);
        sleep(time);
        StopDriving();
    }
    public void ReverseStraightByTime(double power, int time){
        L_M_F.setPower(-power-.1);
        R_M_F.setPower(-power);
        L_M_B.setPower(-power-.1);
        R_M_B.setPower(-power);
        sleep(time);
        StopDriving();
    }
    public void RightTurnStraightByTime(double power, int time){
        R_M_B.setPower(-power-.1);
        L_M_F.setPower(power);
        L_M_B.setPower(power);
        R_M_F.setPower(-power-.1);
        sleep(time);
        StopDriving();
    }
    public void LeftTurnStraightByTime(double power, int time){
        L_M_B.setPower(-power-.1);
        R_M_F.setPower(power);
        R_M_B.setPower(power);
        L_M_F.setPower(-power-.1);
        sleep(time);
        StopDriving();
    }

    protected void ForwardByRotations(double power, double rotations){
        R_M_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R_M_F.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L_M_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L_M_F.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double Target = (rotations*1120);
        telemetry.addData("> Target: ",Target);
        telemetry.update();
        sleep(1000);
        RunWithoutEncoders();
        ForwardStraight(power);
        while ((Math.abs(R_M_B.getCurrentPosition()) < Target && Math.abs(R_M_F.getCurrentPosition()) < Target && Math.abs(L_M_F.getCurrentPosition()) < Target && Math.abs(L_M_B.getCurrentPosition()) < Target)){
           /* R_M_F.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            L_M_F.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            R_M_B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            L_M_B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/
            FourEncodersTelemetry();
            telemetry.update();
        }
        StopDriving();
    }
    protected void ReverseByRotations(double power, double rotations){
        ResetEncoders();
        MotorsReverse();
        double Target = (rotations*1120);
        telemetry.addData("> Target: ",Target);
        telemetry.update();
        sleep(1000);
        RunWithEncoders();
        L_M_F.setPower(power);
        R_M_F.setPower(power);
        L_M_B.setPower(power);
        R_M_B.setPower(power);
        while ((Math.abs(R_M_B.getCurrentPosition()) < Target && Math.abs(R_M_F.getCurrentPosition()) < Target && Math.abs(L_M_F.getCurrentPosition()) < Target && Math.abs(L_M_B.getCurrentPosition()) < Target)){
            RunWithEncoders();
            FourEncodersTelemetry();
            telemetry.update();
        }
        StopDriving();
        MotorsForward();
        RunWithoutEncoders();
    }
    protected void RightWheelsByRotations(double power, double rotations){
        R_M_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R_M_F.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L_M_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L_M_F.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double Target = (rotations*1120);
        telemetry.addData("> Target: ",Target);
        telemetry.update();
        sleep(1000);
        R_M_F.setPower(power);
        R_M_B.setPower(power);
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
    protected void LeftWheelsByRotations(double power, double rotations){
        L_M_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L_M_F.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double Target = (rotations*1120);
        telemetry.addData("> Target: ",Target);
        telemetry.update();
        sleep(1000);
        L_M_F.setPower(power);
        L_M_B.setPower(power);
        while (Math.abs(L_M_F.getCurrentPosition()) < Target && Math.abs(L_M_B.getCurrentPosition()) < Target){
            L_M_F.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            L_M_B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FourEncodersTelemetry();
            telemetry.update();
        }
        L_M_F.setPower(0);
        L_M_B.setPower(0);
    }
    protected void LeftWheelsBackByRotations(double power, double rotations){
        MotorsReverse();
        L_M_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L_M_F.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(500);

        double Target = (rotations*1120);
        telemetry.addData("> Target: ",Target);
        telemetry.update();
        sleep(1000);
        L_M_F.setPower(power);
        L_M_B.setPower(power);
        while (Math.abs(L_M_F.getCurrentPosition()) < Target && Math.abs(L_M_B.getCurrentPosition()) < Target){
            L_M_F.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            L_M_B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FourEncodersTelemetry();
            telemetry.update();
        }
        L_M_F.setPower(0);
        L_M_B.setPower(0);
        MotorsForward();
    }
    protected void RightWheelsBackByRotations(double power, double rotations){
        MotorsReverse();
        R_M_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R_M_F.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L_M_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L_M_F.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double Target = (rotations*1120);
        telemetry.addData("> Target: ",Target);
        telemetry.update();
        sleep(1000);
        R_M_F.setPower(power);
        R_M_B.setPower(power);
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
        MotorsForward();
    }

    public void ForwardByDistance(double power, int targetedDistance){
        while (RangeSensor.getDistance(DistanceUnit.CM) > targetedDistance){
            Forward(power);
            telemetry.addData("Distance: ", RangeSensor.cmUltrasonic());
            telemetry.update();
        }StopDriving();
    }
    public void TurnRightByGyro(int angle, int safetyNet, double power){
        MotorsForward();
        double zTotal = GyroSensor.getIntegratedZValue();
        while (Math.abs(zTotal - angle) > safetyNet) {
            if (zTotal > angle) {
                LeftTurn(power);
            } else if (zTotal < angle) {
                RightTurn(power);
            }
            zTotal = GyroSensor.getIntegratedZValue();
            telemetry.addData("Grados: ", zTotal);
            telemetry.update();
        }
        StopDriving();
        zTotal = GyroSensor.getIntegratedZValue();
        telemetry.addData("Grados: ", zTotal);
        telemetry.update();
    }
    public void TurnLeftByGyro(int angle, int safetyNet, double power){
        MotorsForward();
        double zTotal = GyroSensor.getIntegratedZValue();
        while (Math.abs(zTotal - angle) > safetyNet) {
            if (zTotal < angle) {
                LeftTurn(power);
            } else if (zTotal > angle) {
                RightTurn(power);
            }
            zTotal = GyroSensor.getIntegratedZValue();
            telemetry.addData("Grados: ", zTotal);
            telemetry.update();
        }
        StopDriving();
        zTotal = GyroSensor.getIntegratedZValue();
        telemetry.addData("Grados: ", zTotal);
        telemetry.update();
    }
    protected void TurnByGyro(int angle, int safetyNet, double power){
        MotorsForward();
        RunWithEncoders();

        double zTotal = GyroSensor.getIntegratedZValue();
        while (Math.abs(zTotal - angle) > safetyNet) {
            if (zTotal < angle) {
                L_M_B.setPower(-power);
                R_M_F.setPower(power);
                R_M_B.setPower(power);
                L_M_F.setPower(-power);
            } else if (zTotal > angle) { //EXCHANGE > & < ?
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
    }

    public void CheckJewelColor() {
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
    public void CheckStoneColor() {
        float hsvValues[] = {0, 0, 0};

        Color.RGBToHSV(LineSensor.red() * 8, LineSensor.green() * 8, LineSensor.blue() * 8, hsvValues);
        if (LineSensor.blue() > LineSensor.red() && LineSensor.blue() > LineSensor.green()) {
            StoneColor = blue;
        } else if (LineSensor.red() > LineSensor.blue() && LineSensor.red() > LineSensor.green()) {
            StoneColor = red;
        } else {
            telemetry.addLine(">  NO STONE COLOR DETECTED");
        }
        telemetry.addData(">  Stone Color: ", StoneColor);
        telemetry.update();
    }
    public void CheckGyroValues() {
        telemetry.addData("0", "Heading %03d", GyroSensor.getHeading());
        telemetry.addData("1", "Int. Ang. %03d", GyroSensor.getIntegratedZValue());
        telemetry.addData("2", "X av. %03d", GyroSensor.rawX());
        telemetry.addData("3", "Y av. %03d", GyroSensor.rawY());
        telemetry.addData("4", "Z av. %03d", GyroSensor.rawZ());
        telemetry.update();
    }
    public void CheckRangeDistanceCM() {
        telemetry.addData("cm optical", "%.2f cm", RangeSensor.cmOptical());
        telemetry.addData("cm", "%.2f cm", RangeSensor.getDistance(DistanceUnit.CM));
        telemetry.update();
    }
    public void GyroHeading() {
        telemetry.addData("Heading: ", GyroSensor.getHeading());
        telemetry.update();
    }
    public void VuMarkCheck(){
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
    public void FourEncodersTelemetry(){
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

    public void MotorsForward(){
        R_M_F.setDirection(DcMotor.Direction.FORWARD);
        R_M_B.setDirection(DcMotor.Direction.FORWARD);
        L_M_F.setDirection(DcMotor.Direction.REVERSE);
        L_M_B.setDirection(DcMotor.Direction.REVERSE);
    }
    public void MotorsReverse(){
        R_M_F.setDirection(DcMotor.Direction.REVERSE);
        R_M_B.setDirection(DcMotor.Direction.REVERSE);
        L_M_F.setDirection(DcMotor.Direction.FORWARD);
        L_M_B.setDirection(DcMotor.Direction.FORWARD);
    }
    public void RunWithEncoders(){
        R_M_F.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        R_M_B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        L_M_F.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        L_M_B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void RunWithoutEncoders(){
        R_M_F.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        R_M_B.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        L_M_F.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        L_M_B.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void ResetEncoders(){
        R_M_F.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R_M_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L_M_F.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L_M_B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void LiftGlyph() {
        Lifter.setPower(.5);
        sleep(550);
        Lifter.setPower(.1);
    }
    public void LiftGlyph2() {
        Lifter.setPower(1);
        sleep(1000);
        Lifter.setPower(.1);
    }
    public void TakeDownGlyph(){
        Lifter.setPower(-1);
        sleep(1000);
        Lifter.setPower(0);
    }
    public void GrabGlyph(double power, int time){
        GlyphGrabberR.setPower(power);
        GlyphGrabberL.setPower(power);
        sleep(time);
        GlyphGrabberR.setPower(0);
        GlyphGrabberL.setPower(0);
    }
    public void ReleaseGlyph(double power, int time){
        GrabGlyph(-power,time);
    }
    public void HitJewel(double power, double rotations){
        if (StoneColor == red) {
            R_S_J.setPosition(.6);
            sleep(1000);
            CheckJewelColor();
            sleep(500);
            R_S_J.setPosition(.67);
            sleep(500);
            if (JewelColor == red) {
                RightWheelsBackByRotations(power, rotations);
                sleep(850);
                R_S_J.setPosition(.1);
                sleep(850);
                RightWheelsByRotations(power, rotations);
            } else if (JewelColor == blue) {
                RightWheelsByRotations(power, rotations);
                sleep(850);
                R_S_J.setPosition(.1);
                sleep(850);
                RightWheelsBackByRotations(power, rotations);
            } else {
                telemetry.addLine("ERROR - HIT JEWEL");
                telemetry.update();
                R_S_J.setPosition(.1);
                L_S_J.setPosition(1);
            }
        }
        else if (StoneColor == blue) {
            L_S_J.setPosition(.67);
            sleep(1000);
            CheckJewelColor();
            sleep(500);
            L_S_J.setPosition(.65);
            if (JewelColor == red) {
                LeftWheelsByRotations(power,rotations);
                //  sleep(250);
                L_S_J.setPosition(1);
                // sleep(250);
                LeftWheelsBackByRotations(power, rotations);
            } else if (JewelColor == blue) {
                LeftWheelsBackByRotations(power, rotations);
                // sleep(250);
                L_S_J.setPosition(1);
                // sleep(250);//850
                LeftWheelsByRotations(power, rotations+.1);
            } else {
                telemetry.addLine("ERROR - HIT JEWEL");
                telemetry.update();
                R_S_J.setPosition(.1);
                L_S_J.setPosition(1);
            }
        }else{
            telemetry.addLine("ERROR - HIT JEWEL");
            telemetry.update();
            R_S_J.setPosition(.1);
            L_S_J.setPosition(1);
        }
    }
    public void WayToCryptobox(){
        ForwardByRotations(.5,.85);
        StopDriving();
        sleep(500);
        LiftGlyph2();
        StopDriving();
        sleep(500);
        TurnToCryptoboxBlue(.5,90,3);//target angle:90
        StopDriving();
        sleep(500);
        ForwardByRotations(.3,.6);//Closest rotations:.4 Center rotations: .5 Farest rotations: .6
        StopDriving();
        sleep(500);
        TurnToCryptoboxBlue(.4,0,2);
    }
    public void PlaceGlyphOnCryptobox(){
        //ReverseByRotations(.2,.3);
        TakeDownGlyph();
        if (Multi == "LEFT"){
            TurnToCryptoboxBlue(.5,-30,3);//LEFT
        }
        else if(Multi == "CENTER"){
            TurnToCryptoboxBlue(.5,0,3);
        }
        else if (Multi == "RIGHT"){
            TurnToCryptoboxBlue(.5,30,3);
        }
        else   {
            telemetry.addLine("ERROR - PLACE GLYPH TURN");
            telemetry.update();
        }
        ForwardByDistance(.2,23);
        ReleaseGlyph(1,850);
    }
    public void ParkingMove(){
        ReverseByRotations(.2,.07);
        StopDriving();
        sleep(250);
        TurnToCryptoboxBlue(.5,90,3);
        StopDriving();
        sleep(250);
        if (Multi == "LEFT"){
            ForwardByRotations(.2,05);
        }
        else{
            ReverseByRotations(.2,.05);
        }
    }
    protected void TurnToCryptoboxBlue(double power, int targetAngle, int safetyNet) {
        double zTotal = GyroSensor.getIntegratedZValue();
        while (Math.abs(zTotal - targetAngle) > safetyNet) {
            if (zTotal < targetAngle) {
                L_M_B.setPower(power);
                R_M_F.setPower(-power);
                R_M_B.setPower(-power);
                L_M_F.setPower(power);
            } else if (zTotal > targetAngle) {
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
    }
    protected void TurnToCryptoboxRed(double power, int targetAngle, int safetyNet) {
        double zTotal = GyroSensor.getIntegratedZValue();
        while (Math.abs(zTotal - targetAngle) > safetyNet) {
            if (zTotal > targetAngle) {
                L_M_B.setPower(-power);
                R_M_F.setPower(power);
                R_M_B.setPower(power);
                L_M_F.setPower(-power);
            } else if (zTotal < targetAngle) {
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
        telemetry.update();    }
    public void HitJewelHelp(double rotations){
        L_S_J.setPosition(.67);
        LeftWheelsBackByRotations(.2, rotations);
        sleep(850);
        L_S_J.setPosition(1);
        sleep(850);
        LeftWheelsByRotations(.2, rotations);
    }
    public void LastTurn(){
    }
}
