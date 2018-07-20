        package org.firstinspires.ftc.teamcode.RelicRecovery.Encoders.Testing;


        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorController;
        import com.qualcomm.robotcore.util.ElapsedTime;
        import com.qualcomm.robotcore.util.Range;

        import java.text.SimpleDateFormat;
        import java.util.Date;

/**
 * Created by bk on 9/21/2015.
 */
//@Autonomous(name = "EncodersTest")
public class EncodersTest extends OpMode {
    private String            startDate;
    private ElapsedTime runTime = new ElapsedTime();

    private DcMotorController motorControl1;
    private DcMotor           motor1;

    public EncodersTest() {

    }

    @Override public void init() {
        startDate = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss").format(new Date());

        // The strings must match names given in Settings->Configure Robot
        motorControl1 = hardwareMap.dcMotorController.get("Motor Controller 1");
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
        setDrivePower(-gamepad1.left_stick_y,-gamepad1.right_stick_y);

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