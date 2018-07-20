package org.firstinspires.ftc.teamcode.RelicRecovery.Encoders.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Jorge on 09/11/2017.
 */
//@Autonomous(name = "AS & Test Bby")
public class AS_and_Test_Bby extends LinearOpMode{
    private String            startDate;
    private ElapsedTime runTime = new ElapsedTime();

    private DcMotor motor1;

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;

    @Override
    public void runOpMode(){
        motor1  = hardwareMap.dcMotor.get("motor1");

        motor1.setDirection(DcMotor.Direction.FORWARD);

        // Do not do RESET_ENCODERS and RUN_WITHOUT_ENCODERS
        // If you do - it will not run.
//        setDriveMode(DcMotorController.RunMode.RESET_ENCODERS);
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        setDrivePower(0.0, 0.0);

        runTime.reset();

        waitForStart();
        encoderDrive(DRIVE_SPEED,  10,   5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        //encoderDrive(TURN_SPEED,   12,  4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, -10,  4.0);  // S3: Reverse 24 Inches with 4 Sec timeout
        telemetry.addData("2 Motor 2", motor1.getCurrentPosition());

        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void setDrivePower(double left, double right) {
        // This assumes power is given as -1 to 1
        // The range clip will make sure it is between -1 and 1
        // An incorrect value can cause the program to exception
        motor1.setPower(Range.clip(left, -1.0, 1.0));
    }
    public void setDriveMode(DcMotor.RunMode mode) {
        if (motor1.getMode() != mode) {
            motor1.setMode(mode);
        }
    }
    public void encoderDrive(double speed,double leftInches,double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = motor1.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            motor1.setTargetPosition(newLeftTarget);

            // Turn On RUN_TO_POSITION
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            //runtime.reset();
            runTime.reset();
            motor1.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                   (runTime.seconds() < timeoutS) &&
                    (motor1.isBusy())){

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget);
                telemetry.update();
            }

            // Stop all motion;
            motor1.setPower(0);

            // Turn off RUN_TO_POSITION
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
