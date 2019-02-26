
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static android.os.SystemClock.sleep;

@TeleOp(name = "yeahhh", group = "yeahhh")
public class yeahhh extends OpMode {
    HardwarePushbot robot = new HardwarePushbot();

    static final double COUNTS_PER_MOTOR_REV = 1680;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 0.23622;
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
        double HMP = 0;
        double SlideRotLeftP = 0;
        double SlideRotRightP = 0;
        double SlideLinP = 0;
        double S1P = 0;
        double LSposition;

        LSposition = Lockservo.getPosition();
        if (LSposition != 0) {
            Lockservo.setPosition(0);
        }

        ramp(-gamepad1.left_stick_y, -gamepad1.right_stick_y, -gamepad1.left_stick_y, -gamepad1.right_stick_y);

        if (gamepad1.left_trigger > 0.5) {
            ramp(-gamepad1.left_trigger, gamepad1.left_trigger, gamepad1.left_trigger, -gamepad1.left_trigger);
        } else if (gamepad1.right_trigger > 0.5) {
            ramp(-gamepad1.left_trigger, gamepad1.left_trigger, gamepad1.left_trigger, -gamepad1.left_trigger);
        }

        if (gamepad2.dpad_up) {
            encoderHang(1, -8.3, 2);
        }
        if (gamepad2.dpad_down) {
            HMP = 1;
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
        // Send calculated power to motors
        HM.setPower(HMP);
        SlideRotLeft.setPower(SlideRotLeftP);
        SlideRotRight.setPower(SlideRotRightP);
        SlideLin.setPower(SlideLinP);
        S1.setPower(S1P);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */

    private void ramp(double LFMT, double RFMT, double LBMT, double RBMT) {
        double LFMP = LFM.getPower();
        double RFMP = RFM.getPower();
        double LBMP = LBM.getPower();
        double RBMP = RBM.getPower();

        if (gamepad1.left_bumper && gamepad1.right_bumper) {
            // Slow Mode halves the power to slow down the motors.
            // This is for lining up the robot for hanging in endgame.
            LFMT /= 4;
            RFMT /= 4;
            LBMT /= 4;
            RBMT /= 4;

        }
        if (LFMP != LFMT) {
            LFMP += (LFMT - LFMP) / 4;
        }
        if (RFMP != RFMT) {
            RFMP += (RFMT - RFMP) / 4;
        }
        if (RFMP != LBMT) {
            LBMP += (LBMT - LBMP) / 4;
        }
        if (RFMP != RBMT) {
            RBMP += (RBMT - RBMP) / 4;
        }

        LFM.setPower(LFMP);
        RFM.setPower(RFMP);
        LBM.setPower(LBMP);
        RBM.setPower(RBMP);

        composeTelemetryMotor(LFMP, RFMP, LBMP, RBMP);
    }

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

    private void composeTelemetryMotor(double LFMP, double RFMP, double LBMP, double RBMP){
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Left-Front: ", LFMP);
        telemetry.addData("Right-Front: ", RFMP);
        telemetry.addData("Left-Back: ", LBMP);
        telemetry.addData("Right-Back: ", RBMP);
        telemetry.addLine("Bins: 2");
    }

    @Override
    public void stop() {

    }

}