import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.vision.MasterVision;
import org.firstinspires.ftc.teamcode.vision.SampleRandomizedPositions;

import static android.os.SystemClock.sleep;

@Autonomous
public class SimpleAutonomous extends OpMode {

    HardwarePushbot robot = new HardwarePushbot();

    static final double COUNTS_PER_MOTOR_REV = 1680;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 0.23622;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double INCREMENT = 0.1;
    boolean rampUp = true;

    public ElapsedTime runtime = new ElapsedTime();
    private DcMotor LFM = null;
    private DcMotor RFM = null;
    private DcMotor LBM = null;
    private DcMotor RBM = null;
    private DcMotor HM = null;
    private DcMotor SlideRotLeft = null;
    private DcMotor SlideRotRight = null;
    private DcMotor SlideLin = null;
    private Servo Lockservo = null;
    private int Gold = 0;
    private double LFMP = 0;
    private double RFMP = 0;
    private double LBMP = 0;
    private double RBMP = 0;
    private double HMP = 0;
    private double SlideRotLeftP = 0;
    private double SlideRotRightP = 0;
    private double SlideLinP = 0;
    private double t = System.currentTimeMillis();
    MasterVision vision;
    SampleRandomizedPositions goldPosition;

    long start_time;

    @Override
    public void init() {
        LFM = hardwareMap.get(DcMotor.class, "LFM");
        RFM = hardwareMap.get(DcMotor.class, "RFM");
        LBM = hardwareMap.get(DcMotor.class, "LBM");
        RBM = hardwareMap.get(DcMotor.class, "RBM");
        HM = hardwareMap.get(DcMotor.class, "HM");
        SlideRotLeft = hardwareMap.get(DcMotor.class, "SlideRotLeft");
        SlideRotRight = hardwareMap.get(DcMotor.class, "SlideRotRight");
        SlideLin = hardwareMap.get(DcMotor.class, "SlideLin");
        Lockservo = hardwareMap.get(Servo.class, "Lockservo");

        LFM.setDirection(DcMotor.Direction.REVERSE);
        LBM.setDirection(DcMotor.Direction.REVERSE);
        RFM.setDirection(DcMotor.Direction.FORWARD);
        RBM.setDirection(DcMotor.Direction.FORWARD);


        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;// recommended camera direction
        parameters.vuforiaLicenseKey = "ASrkvw7/////AAABmdfG2uEBx0wAndbotgzIzzRcEHENua4tUnW97NIJE6pEM8HJf1L2Vl2/KxWo7nMQsaaBdEfwq+DCuAVfELk/b/13ybWq6zvUbNDl7g46hdIFGcW2iOds+kdOXmE7NaJCxJS4ytBYwEYX0F4U2VLScBzH0NnsCN+zHbZSg/IRYI2YifEZLYUiLWyZgVyKGkhrx12IjqMdp+t3YU2yXrpnZgsMg5VcZe57P3Vt7i4dhP4EfHvjadR2xfRGhUgXjkAyX3gHkqwcpIsrFDsZEKVuASvpfsjlHM3DA337WuvYP74Z97Rw66MZyz1Ocz02rjWSEllHuNVFGVURayBn8qrGYKR34e3Nw4xGRR9uu8hWCq9T";

        vision = new MasterVision(parameters, hardwareMap, false, MasterVision.TFLiteAlgorithm.INFER_NONE);
        vision.init();// enables the camera overlay. this will take a couple of seconds
        vision.enable();// enables the tracking algorithms. this might also take a little time

    }

    @Override
    public void start() {
        super.start();
        // Save the system clock when start is pressed
        start_time = System.currentTimeMillis();
    }

    @Override
    public void loop() {

        vision.disable();// disables tracking algorithms. this will free up your phone's processing power for other jobs.

        goldPosition = vision.getTfLite().getLastKnownSampleOrder();

        telemetry.addData("Gold Position:", goldPosition);// giving feedback

        switch (goldPosition) { // using for things in the autonomous program
            case LEFT:
                telemetry.addLine("Left case");
                Gold = 1;
                break;
            case CENTER:
                telemetry.addLine("Center case");
                Gold = 2;
                break;
            case RIGHT:
                telemetry.addLine("Right case");
                Gold = 3;
                break;
            case UNKNOWN:
                telemetry.addLine("Skipping sampling");
                break;
        }

        telemetry.update();

        vision.shutdown();

        // Lift rachet while moving hang motor
        if (t < start_time + 1000) {
            Lockservo.setPosition(0);
        }
        if (t < start_time + 1000) {
            HMP = 1;
        }
        // Unhang and stop hang motor
        else if (t < start_time + 4500) {
            encoderHang(8.3);
        }
        // Back up off the lander
        else if (t < start_time + 5000) {
            LFMP = -0.3;
            RFMP = -0.3;
            LBMP = -0.3;
            RBMP = -0.3;
        }
        // Sampling
        else if (t < start_time + 5200) {
            LFMP = 0.5;
            RFMP = -0.5;
            LBMP = 0.5;
            RBMP = -0.5;
        } else if (t < start_time + 21500) {
            if (Gold == 0) {

            } else if (Gold == 1) {
                Left();
            } else if (Gold == 2) {
                Middle();
            } else if (Gold == 3) {
                Right();
            }
        }
        if (System.currentTimeMillis() < start_time + 250) {

        }
        if (System.currentTimeMillis() < start_time + 250) {

        }
        if (System.currentTimeMillis() < start_time + 250) {

        }
        if (System.currentTimeMillis() < start_time + 250) {

        }


        LFM.setPower(LFMP);
        RFM.setPower(RFMP);
        LBM.setPower(LBMP);
        RBM.setPower(RBMP);
        HM.setPower(HMP);
        SlideRotLeft.setPower(SlideRotLeftP);
        SlideRotRight.setPower(SlideRotRightP);
        SlideLin.setPower(SlideLinP);

    }

    private void Left() {
        if (t < start_time + 5500) {
            LFMP = 0.5;
            RFMP = -0.5;
            LBMP = -0.5;
            RBMP = 0.5;
        } else if (t < start_time + 6500) {
            LFMP = 0.5;
            RFMP = 0.5;
            LBMP = 0.5;
            RBMP = 0.5;
        } else if (t < start_time + 11500) {
            LFMP = 1;
            RFMP = -1;
            LBMP = -1;
            RBMP = 1;
        } else if (t < start_time + 21500) {
            LFMP = 1;
            RFMP = 1;
            LBMP = 1;
            RBMP = 1;
        }
    }

    private void Middle() {
        if (t < start_time + 5300) {
            LFMP = 0.5;
            RFMP = -0.5;
            LBMP = -0.5;
            RBMP = 0.5;
        } else if (t < start_time + 10300) {
            LFMP = 1;
            RFMP = -1;
            LBMP = -1;
            RBMP = 1;
        } else if (t < start_time + 10800) {
            LFMP = -0.5;
            RFMP = -0.5;
            LBMP = -0.5;
            RBMP = -0.5;
        } else if (t < start_time + 20800) {
            LFMP = 1;
            RFMP = 1;
            LBMP = 1;
            RBMP = 1;
        } else if (t < start_time + 21500) {
            start_time += 700;
        }
    }

    private void Right() {
        if (t < start_time + 5300) {
            LFMP = 0.5;
            RFMP = -0.5;
            LBMP = -0.5;
            RBMP = 0.5;
        } else if (t < start_time + 6300) {
            LFMP = 0.5;
            RFMP = 0.5;
            LBMP = 0.5;
            RBMP = 0.5;
        } else if (t < start_time + 11300) {
            LFMP = 1;
            RFMP = -1;
            LBMP = -1;
            RBMP = 1;
        } else if (t < start_time + 21300) {
            LFMP = 1;
            RFMP = 1;
            LBMP = 1;
            RBMP = 1;
        } else if (t < start_time + 21500) {
            start_time += 200;
        }
    }

    private void encoderHang(double distance) {
        int newHangTarget;
        // Determine new target position, and pass to motor controller
        newHangTarget = robot.HM.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        robot.HM.setTargetPosition(newHangTarget);


        // reset the timeout time and start motion.
        runtime.reset();
        robot.HM.setPower(Math.abs(0.01));

        // keep looping until motor is at target position
        while ((runtime.seconds() < start_time + 4500) && robot.HM.isBusy()) {
            if (rampUp) {
                // Keep stepping up until we hit the max value.
                HMP += INCREMENT;
                if (robot.HM.getCurrentPosition() >= newHangTarget) {

                    rampUp = !rampUp;   // Switch ramp direction
                }
            }
            HM.setPower(HMP);
            sleep(50);
        }

        // Stop all motion;
        robot.HM.setPower(0);

    }
}