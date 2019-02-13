/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static android.os.SystemClock.sleep;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "yeahhh", group = "yeahhh")
public class yeahhh extends OpMode {
    HardwarePushbot robot = new HardwarePushbot();

    static final double COUNTS_PER_MOTOR_REV = 1680;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 0.23622;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor LFM = null;
    private DcMotor RFM = null;
    private DcMotor LBM = null;
    private DcMotor RBM = null;
    private DcMotor HM = null;
    private DcMotor SlideRotLeft = null;
    private DcMotor SlideRotRight = null;
    private DcMotor SlideLin = null;
    private Servo Lockservo;
    private CRServo S1;
    private boolean driver2 = false;
    private boolean hang = false;

    public void main(String[] args) {

    }

    @Override
    public void init() {
        telemetry.addData("Status:", "Initializing");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        LFM = hardwareMap.get(DcMotor.class, "LFM");
        RFM = hardwareMap.get(DcMotor.class, "RFM");
        LBM = hardwareMap.get(DcMotor.class, "LBM");
        RBM = hardwareMap.get(DcMotor.class, "RBM");
        HM = hardwareMap.get(DcMotor.class, "HM");
        SlideRotLeft = hardwareMap.get(DcMotor.class, "SlideRotLeft");
        SlideRotRight = hardwareMap.get(DcMotor.class, "SlideRotRight");
        SlideLin = hardwareMap.get(DcMotor.class, "SlideLin");
        Lockservo = hardwareMap.get(Servo.class, "Lockservo");
        S1 = hardwareMap.get(CRServo.class, "S1");

        robot.init(hardwareMap);
        robot.HM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.HM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.HM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("Encoders Initialized:", "Starting at %7d", robot.HM.getCurrentPosition());
        telemetry.update();


        // Set zero power behavior to resist motion instead of coast.
        // This allows for more precise stopping with any motors.
        robot.HM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.SlideRotLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.SlideRotRight.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        LFM.setDirection(DcMotor.Direction.REVERSE);
        LBM.setDirection(DcMotor.Direction.REVERSE);
        RFM.setDirection(DcMotor.Direction.FORWARD);
        RBM.setDirection(DcMotor.Direction.FORWARD);
        SlideRotLeft.setDirection(DcMotor.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        double LFMP = 0;
        double RFMP = 0;
        double LBMP = 0;
        double RBMP = 0;
        double HMP = 0;
        double SlideRotLeftP = 0;
        double SlideRotRightP = 0;
        double SlideLinP = 0;
        double S1P = 0;
        double LSposition;

        LSposition = Lockservo.getPosition();
        if (LSposition != 0)

        {
            Lockservo.setPosition(0);
        }

        if (!driver2) {
            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            LFMP = -gamepad1.left_stick_y;
            RFMP = -gamepad1.right_stick_y;
            LBMP = -gamepad1.left_stick_y;
            RBMP = -gamepad1.right_stick_y;

        }
        if (gamepad1.left_trigger > 0.5)

        {
            LFMP = -0.98;
            RFMP = 0.98;
            LBMP = 0.98;
            RBMP = -0.98;
        } else if (gamepad1.right_trigger > 0.5)

        {
            LFMP = 0.98;
            RFMP = -0.98;
            LBMP = -0.98;
            RBMP = 0.98;
        }

        if (gamepad2.dpad_up)

        {
            encoderHang(0.505, -8.3, 2);
        }
        if (gamepad2.dpad_down)

        {
            hang = true;
            encoderHang(0.505, 8.3, 2);
        }

        if (gamepad2.left_trigger > 0.5 && gamepad2.right_trigger > 0.5 && gamepad2.x)

        {
            driver2 = true;
        }
        if (driver2)

        {
            // override
            LFMP = -gamepad2.left_stick_y;
            RFMP = -gamepad2.right_stick_y;
            LBMP = -gamepad2.left_stick_y;
            RBMP = -gamepad2.right_stick_y;

            if (gamepad2.left_trigger > 0.5) {
                LFMP = -0.98;
                RFMP = 0.98;
                LBMP = 0.98;
                RBMP = -0.98;
            }
            if (gamepad2.right_trigger > 0.5) {
                LFMP = 0.98;
                RFMP = -0.98;
                LBMP = -0.98;
                RBMP = 0.98;
            }
        }
        if (gamepad1.left_bumper && gamepad1.right_bumper) {
            // Slow Mode halves the power to slow down the motors.
            // This is for lining up the robot for hanging in endgame.
            LFMP = LFMP / 4;
            RFMP = RFMP / 4;
            LBMP = LBMP / 4;
            RBMP = RBMP / 4;

        }
        if (gamepad2.left_stick_y != 0) {
            if (gamepad2.left_stick_y > 0) {
                SlideRotLeftP = (gamepad2.left_stick_y / 2);
                SlideRotRightP = (gamepad2.left_stick_y / 2);
            }
            if (gamepad2.left_stick_y < 0) {
                SlideRotLeftP = (gamepad2.left_stick_y / 2);
                SlideRotRightP = (gamepad2.left_stick_y / 2);

            }
        }
        if (gamepad2.right_stick_y != 0) {
            if (gamepad2.right_stick_y > 0) {
                SlideLinP = gamepad2.right_stick_y;
            }
            if (gamepad2.right_stick_y < 0) {
                SlideLinP = gamepad2.right_stick_y;
            }
        }
        if (gamepad2.a) {
            S1P = -1;
        }
        if (gamepad2.b) {
            S1P = 1;
        }
        // Send calculated power to wheels
        LFM.setPower(LFMP);
        RFM.setPower(RFMP);
        LBM.setPower(LBMP);
        RBM.setPower(RBMP);
        HM.setPower(HMP);
        SlideRotLeft.setPower(SlideRotLeftP);
        SlideRotRight.setPower(SlideRotRightP);
        SlideLin.setPower(SlideLinP);
        S1.setPower(S1P);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left-front (%.2f), right-front (%.2f), left-back (%.2f), right-back (%.2f)", LFMP, RFMP, LBMP, RBMP);
        String drive_mode = "";
        telemetry.addData("Driving mode", "" + drive_mode);
        telemetry.addLine("Bins: 2");
        telemetry.addData("Position", robot.HM.getCurrentPosition());
        telemetry.addData("Target", robot.HM.getCurrentPosition() + (7 * COUNTS_PER_INCH));
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */


    private void encoderHang(double hangSpeed, double distance, double timeout) {
        int newHangTarget;
        // Determine new target position, and pass to motor controller
        newHangTarget = robot.HM.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        robot.HM.setTargetPosition(newHangTarget);


        // reset the timeout time and start motion.
        runtime.reset();
        robot.HM.setPower(Math.abs(hangSpeed));

        // keep looping until motor is at target position
        while ((runtime.seconds() < timeout) && robot.HM.isBusy()) {

        }

        if (hang) {
            Lockservo.setPosition(1);
            sleep(3000);
            hang = false;
        }

        // Stop all motion;
        robot.HM.setPower(0);

    }

    @Override
    public void stop() {

    }

}