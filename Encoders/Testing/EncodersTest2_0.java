package org.firstinspires.ftc.teamcode.RelicRecovery.Encoders.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.text.SimpleDateFormat;
import java.util.Date;

import javax.crypto.spec.OAEPParameterSpec;

/**
 * Created by Jorge on 09/11/2017.
 */
//@Autonomous(name = "EncodersTest2_0")
public class EncodersTest2_0 extends OpMode{
    private String            startDate;
    private ElapsedTime runTime = new ElapsedTime();

    private DcMotorController motorControl1;
    private DcMotor motor1;

    @Override public void init() {
        startDate = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss").format(new Date());

        // The strings must match names given in Settings->Configure Robot
//        motorControl1 = hardwareMap.dcMotorController.get("Motor Controller 1");
        motor1  = hardwareMap.dcMotor.get("motor1");

        motor1.setDirection(DcMotor.Direction.FORWARD);

        // Do not do RESET_ENCODERS and RUN_WITHOUT_ENCODERS
        // If you do - it will not run.
//        setDriveMode(DcMotorController.RunMode.RESET_ENCODERS);
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        setDrivePower(0.0, 0.0);

        runTime.reset();
    }

    @Override public void loop() {
        // Negative is up on the joystick,
        // Positive is down on the joystick,
        // Send the "negative" of the joystick so up is now positive
        setDrivePower(.1,-gamepad1.right_stick_y);

        telemetry.addData("2 Motor 2", motor1.getCurrentPosition());
    }

    @Override public void stop() {
        setDrivePower(0.0, 0.0);
    }

    /**
     * Set the power to left and right motors, the values must range
     * between -1 and 1.
     * @param left
     * @param right
     */
    public void setDrivePower(double left, double right) {
        // This assumes power is given as -1 to 1
        // The range clip will make sure it is between -1 and 1
        // An incorrect value can cause the program to exception
        motor1.setPower(Range.clip(left, -1.0, 1.0));
    }
   /*
    public void encoderDrive(double speed,

                             double leftInches, double rightInches,
                             double timeoutS) {
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
            runtime.reset();
            motor1.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motor1.isBusy() &&.isBusy())){

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.update();
            }

            // Stop all motion;
            motor1.setPower(0);

            // Turn off RUN_TO_POSITION
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    */

            //  sleep(250);   // optional pause after each move



    /**
     * Sets the drive mode for each motor.
     * The types of Run Modes are
     *   DcMotorController.RunMode.RESET_ENCODERS
     *      Resets the Encoder Values to 0
     *   DcMotorController.RunMode.RUN_TO_POSITION
     *      Runs until the encoders are equal to the target position
     *   DcMotorController.RunMode.RUN_USING_ENCODERS
     *      Attempts to keep the robot running straight utilizing
     *      the PID the reduces the maximum power by about 15%
     *   DcMotorController.RunMode.RUN_WITHOUT_ENCODERS
     *      Applies the power directly
     * @param mode
     */
    public void setDriveMode(DcMotor.RunMode mode) {
        if (motor1.getMode() != mode) {
            motor1.setMode(mode);
        }
    }
}

