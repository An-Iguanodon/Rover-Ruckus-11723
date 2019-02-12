package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

@TeleOp
public class VisionJavaExample extends LinearOpMode{
    MasterVision vision;
    SampleRandomizedPositions goldPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;// recommended camera direction
        parameters.vuforiaLicenseKey = "ASrkvw7/////AAABmdfG2uEBx0wAndbotgzIzzRcEHENua4tUnW97NIJE6pEM8HJf1L2Vl2/KxWo7nMQsaaBdEfwq+DCuAVfELk/b/13ybWq6zvUbNDl7g46hdIFGcW2iOds+kdOXmE7NaJCxJS4ytBYwEYX0F4U2VLScBzH0NnsCN+zHbZSg/IRYI2YifEZLYUiLWyZgVyKGkhrx12IjqMdp+t3YU2yXrpnZgsMg5VcZe57P3Vt7i4dhP4EfHvjadR2xfRGhUgXjkAyX3gHkqwcpIsrFDsZEKVuASvpfsjlHM3DA337WuvYP74Z97Rw66MZyz1Ocz02rjWSEllHuNVFGVURayBn8qrGYKR34e3Nw4xGRR9uu8hWCq9T";

        vision = new MasterVision(parameters, hardwareMap, true, MasterVision.TFLiteAlgorithm.INFER_RIGHT);
        vision.init();// enables the camera overlay. this will take a couple of seconds
        vision.enable();// enables the tracking algorithms. this might also take a little time

        waitForStart();

        vision.disable();// disables tracking algorithms. this will free up your phone's processing power for other jobs.

        goldPosition = vision.getTfLite().getLastKnownSampleOrder();

        while(opModeIsActive()){
            telemetry.addData("goldPosition was", goldPosition);// giving feedback

            switch (goldPosition){ // using for things in the autonomous program
                case LEFT:
                    telemetry.addLine("going to the left");
                    break;
                case CENTER:
                    telemetry.addLine("going straight");
                    break;
                case RIGHT:
                    telemetry.addLine("going to the right");
                    break;
                case UNKNOWN:
                    telemetry.addLine("staying put");
                    break;
            }

            telemetry.update();
        }

        vision.shutdown();
    }
}
