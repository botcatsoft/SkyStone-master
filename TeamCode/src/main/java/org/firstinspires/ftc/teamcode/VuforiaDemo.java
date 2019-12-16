package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

/*
 * This OpMode was written for the VuforiaDemo Basics video. This demonstrates basic principles of
 * using VuforiaDemo in FTC.
 */
@Autonomous(name = "Vuforia")
@Disabled
public class VuforiaDemo extends LinearOpMode
{
    OpenGLMatrix lastLocation = null;
    // Variables to be used for later
    private VuforiaLocalizer vuforiaLocalizer;
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaTrackables visionTargets;

    private VuforiaTrackable target1;
    private VuforiaTrackable target2;
    private VuforiaTrackable target3;
    private VuforiaTrackable target4;
    private VuforiaTrackable target5;
    private VuforiaTrackable target6;
    private VuforiaTrackable target7;
    private VuforiaTrackable target8;
    private VuforiaTrackable target9;
    private VuforiaTrackable target10;
    private VuforiaTrackable target11;
    private VuforiaTrackable target12;
    private VuforiaTrackable target13;




    //target1 = new allTargets[0];


    private VuforiaTrackableDefaultListener listener;

    private OpenGLMatrix lastKnownLocation;
    private OpenGLMatrix phoneLocation;

    private static final String VUFORIA_KEY = "AVop5Pr/////AAABmYh62mTDYUgSsC1tOVg8A0IjuyqMzLO1I9Ue7E0egk11CAzRZeX8GW2ytNBbBI96DbCQ306sVAjACJXwPhaiHFG5dHPRk3daxk2TPpzhXHEinXui59HAdySDhWwyxKSyBc7tkFk7cR2JOThtf5Qy7CJgp3qa8MxpZ2ZhCt433Iji1nMtL69cC9QVA260v7XJkWcXrGomJQYgrZl3w4AkQNrzkUSejo7LBHcaGyy2SHEMppAt8KXvSlDYJJgm737AVy27wgdGoMcS45F5VrAHhaUxy1os4ZDxfm2eNCFyCfzUc+M8jsewh2RZvk98rIbpGyAv7ZVqppF+G9HqQThn+lEGNm0Jy0nEHYajG0pHxVI4"; // Insert your own key here

    private float robotX = 0;
    private float robotY = 0;
    private float robotAngle = 0;
    VuforiaLocalizer vuforia;

    public void runOpMode() throws InterruptedException
    {
        setupVuforia();

        // We don't know where the robot is, so set it to the origin
        // If we don't include this, it would be null, which would cause errors later on
        lastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        VuforiaTrackables skystone = this.vuforia.loadTrackablesFromAsset("Skystone");
        VuforiaTrackable targetElement = skystone.get(0);
        targetElement.setName("TargetElement");  // Stones

        List<VuforiaTrackable> allTargets = new ArrayList<VuforiaTrackable>();
        allTargets.addAll(skystone);

        //VuforiaTrackable[] allTargets;
        /*
        allTargets = new VuforiaTrackable[13];
        allTargets[0] = target1;
        allTargets[1] = target2;
        allTargets[2] = target3;
        allTargets[3] = target4;
        allTargets[4] = target5;
        allTargets[5] = target6;
        allTargets[6] = target7;
        allTargets[7] = target8;
        allTargets[8] = target9;
        allTargets[9] = target10;
        allTargets[10] = target11;
        allTargets[11] = target12;
        allTargets[12] = target13;*/

        waitForStart();

        // Start tracking the targets
        visionTargets.activate();

        while(opModeIsActive())
        {
            for (VuforiaTrackable trackable : allTargets) {
                /**
                 * getUpdatedRobotLocation() will return null if no new information is available since
                 * the last time that call was made, or if the trackable is not currently visible.
                 * getRobotLocation() will return null if the trackable is not currently visible.
                 */
                telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
            }
            /**
             * Provide feedback as to where the robot was last located (if we know).
             */
            if (lastLocation != null) {
                //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
                //telemetry.addData("Pos", format(lastLocation));
            } else {
                telemetry.addData("Pos", "Unknown");
            }
            telemetry.update();
            // Ask the listener for the latest information on where the robot is
            /*OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();

            // The listener will sometimes return null, so we check for that to prevent errors
            if(latestLocation != null)
                lastKnownLocation = latestLocation;

            float[] coordinates = lastKnownLocation.getTranslation().getData();

            robotX = coordinates[0];
            robotY = coordinates[1];
            robotAngle = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

            for(int i = 0; i < 13; i++){


            }
            // Send information about whether the target is visible, and where the robot is
            telemetry.addData("Tracking " + target1.getName(), listener.isVisible());
            telemetry.addData("Last Known Location", formatMatrix(lastKnownLocation));

            // Send telemetry and idle to let hardware catch up
            telemetry.update();
            idle();*/


        }
    }

    private void setupVuforia()
    {
        // Setup parameters to create localizer
        parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId); // To remove the camera view from the screen, remove the R.id.cameraMonitorViewId
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.useExtendedTracking = false;
        vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);

        // These are the vision targets that we want to use
        // The string needs to be the name of the appropriate .xml file in the assets folder
        visionTargets = vuforiaLocalizer.loadTrackablesFromAsset("Skystone");
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        for(int i = 0; i < 13; i++) {

        }

        // Setup the target to be tracked
        /*
        targetElement = visionTargets.get(0); // 0 corresponds to the wheels target
        targetElement.setLocation(createMatrix(0, 0, 0, 90, 0, 0));

        allTargets[1] = visionTargets.get(1); // 1 corresponds to the wheels target
        allTargets[1].setName("BridgeBlueRear");
        allTargets[1].setLocation(createMatrix(0, 500, 0, -90, 0, -90));

        allTargets[2] = visionTargets.get(2); // 2 corresponds to the wheels target
        allTargets[2].setName("BridgeRedRear");
        allTargets[2].setLocation(createMatrix(500, 0, 0, 180, -90, 0));

        allTargets[3] = visionTargets.get(3); // 3 corresponds to the wheels target
        allTargets[3].setName("BridgeRedFront");
        allTargets[3].setLocation(createMatrix(500, 1000, 0, 0, 90, 0));

        allTargets[4] = visionTargets.get(4); // 0 corresponds to the wheels target
        allTargets[4].setName("BridgeBlueFront");
        allTargets[4].setLocation(createMatrix(0, 500, 0, 90, 0, 90));

        allTargets[5] = visionTargets.get(5); // C3PO
        allTargets[5].setName("RedPerimeterTgt1");
        allTargets[5].setLocation(createMatrix(1000, 255, 0, 90, 0, 90));

        allTargets[6] = visionTargets.get(6); // Yellow Submarine
        allTargets[6].setName("RedPerimeterTgt2");
        allTargets[6].setLocation(createMatrix(1000, 745, 0, 90, 0, 90));

        allTargets[7] = visionTargets.get(7); // Loch Ness C3PO
        allTargets[7].setName("FrontPerimeterTgt1");
        allTargets[7].setLocation(createMatrix(745, 0, 0, 90, 180, 0));

        allTargets[8] = visionTargets.get(8); // R2-88
        allTargets[8].setName("FrontPerimeterTgt2");
        allTargets[8].setLocation(createMatrix(255, 0, 0, 90, 180, 0));

        allTargets[9] = visionTargets.get(9); // BB8
        allTargets[9].setName("BluePerimeterTgt1");
        allTargets[9].setLocation(createMatrix(0, 255, 0, -90, 0, -90));

        allTargets[10] = visionTargets.get(10); // Good Student
        allTargets[10].setName("BluePerimeterTgt2");
        allTargets[10].setLocation(createMatrix(0, 745, 0, -90, 0, -90));

        allTargets[11] = visionTargets.get(11); // DJ bot
        allTargets[11].setName("RearPerimeterTgt1");
        allTargets[11].setLocation(createMatrix(255, 1000, 0, 180, -90, 0));

        allTargets[12] = visionTargets.get(12); // 3 corresponds to the wheels target
        allTargets[12].setName("RearPerimeterTgt2");
        allTargets[12].setLocation(createMatrix(745, 1000, 0, 0, 90, 0));*/

        // Set phone location on robot
        phoneLocation = createMatrix(0, 225, 0, 90, 0, 0);

        // Setup listener and inform it of phone information
        listener = (VuforiaTrackableDefaultListener) target1.getListener();
        listener.setPhoneInformation(phoneLocation, parameters.cameraDirection);
    }

    // Creates a matrix for determining the locations and orientations of objects
    // Units are millimeters for x, y, and z, and degrees for u, v, and w
    private OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w)
    {
        return OpenGLMatrix.translation(x, y, z).
                multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    // Formats a matrix into a readable string
    private String formatMatrix(OpenGLMatrix matrix)
    {
        return matrix.formatAsTransform();
    }
}