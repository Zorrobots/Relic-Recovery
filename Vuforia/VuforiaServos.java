package org.firstinspires.ftc.teamcode.RelicRecovery.Vuforia;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


/**
 * Created by Jorge on 23/11/2017.
 */
//@Autonomous(name="VuforiaServos")
public class VuforiaServos extends LinearOpMode {
    //WHEELS
    public DcMotor Left_motors = null;
    public DcMotor Right_motors = null;

    public Servo LeftJServo = null;
    public Servo RightJServo = null;
    
    String Multi;


    //VUFORIA
    VuforiaLocalizer vuforia;
    @Override
    public void runOpMode() throws InterruptedException {
        Left_motors = hardwareMap.dcMotor.get("Left_motors");
        Right_motors = hardwareMap.dcMotor.get("Right_motors");

        RightJServo = hardwareMap.servo.get("RightJServo");
        LeftJServo = hardwareMap.servo.get("LeftJServo");
        
        Left_motors.setDirection(DcMotor.Direction.REVERSE);

        //VUFORIA INICIO
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AYy3kgb/////AAAAGfahknPNfEOHpk9drxT3x5s+h85enQDuwX5Y/R9chthrPe1AQ1A+iYyS9PoUpVOVcu4TM/lzJa/PqlyaHKJWh+fI63xLIftsjqQ15b+MoQNZrgG4sw0swD9/yYSfSn3AU6PuQ6OozHZf4zrEOiL2AL/1OMMxbd9KddgiIIX5X/rnx7VFMFiNR8vq+otCHameCqnzdRcCkp1rqo+bewMyMYjTeYIyl29wn0oElYjg1PdBoYgDiUIjQu4sVECgCH7c6+pmEYe37ypfeMCxoGmG60L8bUmq5RrzZ1mxdJkugZ4hRbG/UIm1aHApSHE+ljsAexK3crM78qRfdVK6B9PTsnEEq9C40FuYu/ZqcCglO5VZ\n";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        relicTrackables.activate();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        while (!isStopRequested() && vuMark == RelicRecoveryVuMark.UNKNOWN)  {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            telemetry.addData("VuMark", "%s visible", vuMark);
            sleep(250);
            idle();
        }

        if (vuMark == RelicRecoveryVuMark.LEFT){
            Multi = "LEFT";
        }
        else if (vuMark == RelicRecoveryVuMark.RIGHT){
            Multi = "RIGHT";
        }
        else if (vuMark == RelicRecoveryVuMark.CENTER){
            Multi = "CENTER";
        }

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        waitForStart();
        VuMarkCheckandMoveServos();
        }


    public void Forward (double power){
        Right_motors.setPower(power);
        Left_motors.setPower(power);
    }
    public void VuMarkCheckandRun(){
        if (Multi=="LEFT") {
            telemetry.addLine("Izquierda");
            telemetry.update();
            Forward(.3);
            sleep(500);
            Forward(0);
            sleep(1500);
        }
        else if (Multi=="RIGHT") {
            telemetry.addLine("Derecha");
            telemetry.update();
            Forward(.3);
            sleep(500);
            Forward(0);
            sleep(1500);
        }
        if (Multi=="CENTER") {
            telemetry.addLine("Centro");
            telemetry.update();
            Forward(.3);
            sleep(500);
            Forward(0);
            sleep(1500);
        }
        else{
            telemetry.addLine("Fallaste en VuMarkCheck and Run");
            telemetry.update();
            Forward(0);
            sleep(1000);
        }
    }
    public void VuMarkCheckandMoveServos(){
        if (Multi=="LEFT") {
            telemetry.addLine("Izquierda");
            telemetry.update();
            LeftJServo.setPosition(.5);
            sleep(1000);
            LeftJServo.setPosition(0);
            sleep(1000);
        }
        else if (Multi=="RIGHT") {
            telemetry.addLine("Derecha");
            telemetry.update();
            RightJServo.setPosition(.5);
            sleep(1000);
            RightJServo.setPosition(0);
            sleep(1000);
        }
        if (Multi=="CENTER") {
            telemetry.addLine("Centro");
            telemetry.update();
            RightJServo.setPosition(.5);
            LeftJServo.setPosition(.5);
            sleep(1000);
            RightJServo.setPosition(0);
            LeftJServo.setPosition(0);
            sleep(1000);
        }
        else{
            telemetry.addLine("Fallaste en VuMarkCheck and Run");
            telemetry.update();
            Forward(0);
            sleep(1000);
        }
    }
    }
