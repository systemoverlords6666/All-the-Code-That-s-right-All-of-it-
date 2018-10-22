package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

@Autonomous (name = "Autonomous_6666" ,  group = "sensor")
public class Autonomous_6666 extends LinearOpMode {


    //VuForia
    private static final String VUFORIA_KEY = " AftOfUH/////AAABmfHQ2PnhyUltsx5fIsmFJ1YV/zibssSiPsNVcVrP2Ggre0S6BwhjjZhVlUVe6PQ5jKk1g9ys9Z5nd81xFqyP7Pyg072BJpsj3jQcxkMxK0E8bXcqqctYkPVfvEhh/GlDssHzfHq812FlVMepvkxF2xLzL9jhhhbYhjm9nDlMRJb8oW6tANHjZRJ7LOyDi2QJClST1SwuLDwgCpif6UyXOJ7bXulirIWAJL4LED5kNl8qTsGYRteYwsrxA+JkwHN5pMbUgofWfcrVRcxMJcep4IYCDFQbsgct3wqsLnGSo6n5WbfyLanh9azncmd/l1Kt+t8pAXU0wVNU3L1BrOZ3isJo2psseqZkdeGzNGQZPcP";

    private static final float mmPerInch = 25.4f;
    private static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;
    private static final float mmTargetHeight = (6) * mmPerInch;

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;

    VuforiaLocalizer vuforia;

    boolean BlueRover;
    boolean RedFootprint;
    boolean FrontCraters;
    boolean BackSpace;
    double sensor_ods;

    //Vuforia

    // Motion
    DcMotor lm;
    DcMotor rm;
    DcMotor up;
    //motion

    // Optical Distance Sensor
    OpticalDistanceSensor odsSensor;
    //Optical Distance sensor

    @Override
    public void runOpMode() throws InterruptedException {

        //Movement
        lm = hardwareMap.dcMotor.get("lm");
        rm = hardwareMap.dcMotor.get("rm");
        up = hardwareMap.dcMotor.get("up");

        //up.setDirection(DcMotor.Direction.REVERSE);
        rm.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();

        up.setPower(1);
        sleep(750);

        lm.setPower(-.75);
        rm.setPower(.75);
        sleep(3000);
        //movements

        //vuforia

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);


        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;


        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");

        VuforiaTrackables Nav1 = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackables Nav2 = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackables Nav3 = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackables Nav4 = this.vuforia.loadTrackablesFromAsset("RoverRuckus");

        VuforiaTrackable blueRover = Nav1.get(0);
        blueRover.setName("BlueRover");
        VuforiaTrackable redFootprint = Nav2.get(1);
        redFootprint.setName("RedFootprint");
        VuforiaTrackable frontCraters = Nav3.get(2);
        frontCraters.setName("FrontCraters");
        VuforiaTrackable backSpace = Nav4.get(3);
        backSpace.setName("BackSpace");


        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(Nav1);

        List<VuforiaTrackable> allTrackables1 = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(Nav2);

        List<VuforiaTrackable> allTrackables2 = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(Nav3);

        List<VuforiaTrackable> allTrackables3 = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(Nav4);


        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);


        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);


        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
        frontCraters.setLocation(frontCratersLocationOnField);


        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);


        /**
         * Create a transformation matrix describing where the phone is on the robot.
         *
         * The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
         * pointing to the LEFT side of the Robot.  It's very important when you test this code that the top of the
         * camera is pointing to the left side of the  robot.  The rotation angles don't work if you flip the phone.
         *
         * If using the rear (High Res) camera:
         * We need to rotate the camera around it's long axis to bring the rear camera forward.
         * This requires a negative 90 degree rotation on the Y axis
         *
         * If using the Front (Low Res) camera
         * We need to rotate the camera around it's long axis to bring the FRONT camera forward.
         * This requires a Positive 90 degree rotation on the Y axis
         *
         * Next, translate the camera lens to where it is on the robot.
         * In this example, it is centered (left to right), but 110 mm forward of the middle of the robot, and 200 mm above ground level.
         */

        final int CAMERA_FORWARD_DISPLACEMENT = 110;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        for (VuforiaTrackable trackable : allTrackables1) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        for (VuforiaTrackable trackable : allTrackables2) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        for (VuforiaTrackable trackable : allTrackables3) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        /** Start tracking the data sets we care about. */
        Nav1.activate();
        Nav2.activate();
        Nav3.activate();
        Nav4.activate();

        while (opModeIsActive()) {

            // check all the trackable target to see which one (if any) is visible.
            targetVisible = BlueRover;
            for (VuforiaTrackable trackable : Nav1) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());


                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }

                if (targetVisible = BlueRover) {

                    lm.setPower(.75);
                    rm.setPower(-.75);
                    sleep(2500);

                    lm.setPower(1);
                    rm.setPower(1);
                    sleep(7500);

                    lm.setPower(1);
                    lm.setPower(1);
                    sleep(150);

                    // servo drops the team marker

                }

                // Provide feedback as to where the robot is located (if we know).
                if (targetVisible = BlueRover) {
                    // express position (translation) of robot in inches.
                    VectorF translation = lastLocation.getTranslation();
                    telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                            translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                    // express the rotation of the robot in degrees.
                    Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                    telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                } else {
                    telemetry.addData("Visible Target", "none");
                }
                telemetry.update();
            }

            while (opModeIsActive()) {

                // check all the trackable target to see which one (if any) is visible.
                targetVisible = FrontCraters;
                for (VuforiaTrackable trackable : Nav3) {
                    if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                        telemetry.addData("Visible Target", trackable.getName());


                        // getUpdatedRobotLocation() will return null if no new information is available since
                        // the last time that call was made, or if the trackable is not currently visible.
                        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                        if (robotLocationTransform != null) {
                            lastLocation = robotLocationTransform;
                        }
                        break;
                    }

                    if (targetVisible = FrontCraters = true) {



                    }

                }

                // Provide feedback as to where the robot is located (if we know).
                if (targetVisible = BlueRover) {
                    // express position (translation) of robot in inches.
                    VectorF translation = lastLocation.getTranslation();
                    telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                            translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                    // express the rotation of the robot in degrees.
                    Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                    telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                } else {
                    telemetry.addData("Visible Target", "none");
                }
                telemetry.update();

                // Provide feedback as to where the robot is located (if we know).
                if (targetVisible = FrontCraters) {
                    // express position (translation) of robot in inches.
                    VectorF translation = lastLocation.getTranslation();
                    telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                            translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                    // express the rotation of the robot in degrees.
                    Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                    telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                } else {
                    telemetry.addData("Visible Target", "none");
                }
                telemetry.update();
            }
            //vuforia

            //Optical Distance Sensor start

            // get a reference to our Light Sensor object.
            odsSensor = hardwareMap.get(OpticalDistanceSensor.class, "sensor_ods");

            // wait for the start button to be pressed.
            waitForStart();

            // while the op mode is active, loop and read the light levels.
            // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
          //  while (opModeIsActive()) {

                // send the info back to driver station using telemetry function.
                //telemetry.addData("Raw", odsSensor.getRawLightDetected());
                //telemetry.addData("Normal", odsSensor.getLightDetected());

                //telemetry.update();

               // if (sensor_ods =) {// Type Gold Values//


              //  }
             //   if (sensor_ods =) { // Type silver Values


                //}
            //}

            //Optical Distance Sensor end

        }
    }
}

