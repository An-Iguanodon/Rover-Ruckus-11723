import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.vision.MasterVision;
import org.firstinspires.ftc.teamcode.vision.SampleRandomizedPositions;

@Autonomous
public class SimpleAutonomous extends OpMode {

    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor LFM = null;
    public DcMotor RFM = null;
    public DcMotor LBM = null;
    public DcMotor RBM = null;
    public DcMotor HM = null;
    public DcMotor SlideRotLeft = null;
    public DcMotor SlideRotRight = null;
    public DcMotor SlideLin = null;
    public Servo S1;
    private int Gold = 0;
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
        S1 = hardwareMap.servo.get("S1");

        LFM.setDirection(DcMotor.Direction.REVERSE);
        LBM.setDirection(DcMotor.Direction.REVERSE);
        RFM.setDirection(DcMotor.Direction.FORWARD);
        RBM.setDirection(DcMotor.Direction.FORWARD);


        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;// recommended camera direction
        parameters.vuforiaLicenseKey = "ASrkvw7/////AAABmdfG2uEBx0wAndbotgzIzzRcEHENua4tUnW97NIJE6pEM8HJf1L2Vl2/KxWo7nMQsaaBdEfwq+DCuAVfELk/b/13ybWq6zvUbNDl7g46hdIFGcW2iOds+kdOXmE7NaJCxJS4ytBYwEYX0F4U2VLScBzH0NnsCN+zHbZSg/IRYI2YifEZLYUiLWyZgVyKGkhrx12IjqMdp+t3YU2yXrpnZgsMg5VcZe57P3Vt7i4dhP4EfHvjadR2xfRGhUgXjkAyX3gHkqwcpIsrFDsZEKVuASvpfsjlHM3DA337WuvYP74Z97Rw66MZyz1Ocz02rjWSEllHuNVFGVURayBn8qrGYKR34e3Nw4xGRR9uu8hWCq9T";

        vision = new MasterVision(parameters, hardwareMap, true, MasterVision.TFLiteAlgorithm.INFER_RIGHT);
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
        double LFMP = 0;
        double RFMP = 0;
        double LBMP = 0;
        double RBMP = 0;
        double HMP = 0;
        double SlideRotLeftP = 0;
        double SlideRotRightP = 0;
        double SlideLinP = 0;

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
        // If we're still with the first 3 seconds after pressing start keep driving forward
        if (System.currentTimeMillis() < start_time + 250) {
            S1.setPosition(0);
        }
        if (System.currentTimeMillis() < start_time + 750) {
            HMP = -0.5;
        }
        if (System.currentTimeMillis() < start_time + 2000) {
            if (Gold == 0){

            }else if (Gold == 1){
                if (System.currentTimeMillis() < start_time + 2400 ) {
                    LFMP = -1;
                    RFMP = -1;
                    LBMP = -1;
                    RBMP = -1;
                }
                if (System.currentTimeMillis() < start_time + 2800) {
                    LFMP = 0.98;
                    RFMP = -0.98;
                    LBMP = -0.98;
                    RBMP = 0.98;
                }
            }else if (Gold == 2){
                start_time = start_time + 400;
                if (System.currentTimeMillis() < start_time + 2800) {
                    LFMP = 0.98;
                    RFMP = -0.98;
                    LBMP = -0.98;
                    RBMP = 0.98;
                }
            }else if (Gold == 3) {
                if (System.currentTimeMillis() < start_time + 2400 ) {
                    LFMP = 1;
                    RFMP = 1;
                    LBMP = 1;
                    RBMP = 1;
                }
                if (System.currentTimeMillis() < start_time + 2800) {
                    LFMP = 0.98;
                    RFMP = -0.98;
                    LBMP = -0.98;
                    RBMP = 0.98;
                }
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
}