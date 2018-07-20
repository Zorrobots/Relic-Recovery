package org.firstinspires.ftc.teamcode.RelicRecovery.Vuforia;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


/**
 * Created by Jorge on 23/11/2017.
 */
//@Autonomous(name="AutoRevTry1")
public class AutoRevTry1 extends LinearOpMode {
    DcMotor motorLeftF;
    DcMotor motorLeftB;
    DcMotor motorRightF;
    DcMotor motorRightB;

    //VUFORIA
    VuforiaLocalizer vuforia;
    @Override
    public void runOpMode() throws InterruptedException {
        motorLeftB = hardwareMap.dcMotor.get("motorLeftB");
        motorLeftF = hardwareMap.dcMotor.get("motorLeftF");
        motorRightB = hardwareMap.dcMotor.get("motorRightB");
        motorRightF = hardwareMap.dcMotor.get("motorRightF");
        motorLeftF.setDirection(DcMotor.Direction.REVERSE);
        motorLeftB.setDirection(DcMotor.Direction.REVERSE);

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

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        //VUFORIA FINAL

        waitForStart();

        //VUFORIA INICIO
//   relicTrackables.activate();
        //VUFORIA FINAL

        while (opModeIsActive()) {
            if (vuMark==RelicRecoveryVuMark.LEFT) {
                telemetry.addLine("Izquierda");
                telemetry.update();
                Forward(.3);
                sleep(500);
                Forward(0);
                sleep(1500);
            }
            else if (vuMark==RelicRecoveryVuMark.RIGHT) {
                telemetry.addLine("Derecha");
                telemetry.update();
                Forward(.3);
                sleep(500);
                Forward(0);
                sleep(1500);
            }
            if (vuMark==RelicRecoveryVuMark.CENTER) {
                telemetry.addLine("Centro");
                telemetry.update();
                Forward(.3);
                sleep(500);
                Forward(0);
                sleep(1500);
            }
            else{
                telemetry.addLine("Fallaste");
                telemetry.update();
                Forward(0);
                sleep(1000);
            }
        }
    }
    public void Forward (double power){
        motorRightF.setPower(power);
        motorRightB.setPower(power);
        motorLeftB.setPower(power);
        motorLeftF.setPower(power);
    }
}
