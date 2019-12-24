package org.firstinspires.ftc.teamcode.OpModes.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.OpModes.Abstract.BaseOpMode;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="BlueBuildAuto", group ="Concept")
public class BlueBuildAuto extends BaseOpMode {
    public static final String TAG = "Vuforia Navigation Sample";

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    @Override public void runOpMode() {


        //Set Motors
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code on the next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "AVop5Pr/////AAABmYh62mTDYUgSsC1tOVg8A0IjuyqMzLO1I9Ue7E0egk11CAzRZeX8GW2ytNBbBI96DbCQ306sVAjACJXwPhaiHFG5dHPRk3daxk2TPpzhXHEinXui59HAdySDhWwyxKSyBc7tkFk7cR2JOThtf5Qy7CJgp3qa8MxpZ2ZhCt433Iji1nMtL69cC9QVA260v7XJkWcXrGomJQYgrZl3w4AkQNrzkUSejo7LBHcaGyy2SHEMppAt8KXvSlDYJJgm737AVy27wgdGoMcS45F5VrAHhaUxy1os4ZDxfm2eNCFyCfzUc+M8jsewh2RZvk98rIbpGyAv7ZVqppF+G9HqQThn+lEGNm0Jy0nEHYajG0pHxVI4";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        /**
         * Instantiate the Vuforia engine
         */
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        /**
         * Load the data sets that for the trackable objects we wish to track. These particular data
         * sets are stored in the 'assets' part of our application (you'll see them in the Android
         * Studio 'Project' view over there on the left of the screen). You can make your own datasets
         * with the Vuforia Target Manager: https://developer.vuforia.com/target-manager. PDFs for the
         * example "StonesAndChips", datasets can be found in in this project in the
         * documentation directory.
         */
        VuforiaTrackables skystone = this.vuforia.loadTrackablesFromAsset("Skystone");
        VuforiaTrackable redTarget1 = skystone.get(5);
        redTarget1.setName("RedPerimeterTgt1");

        VuforiaTrackable redTarget2  = skystone.get(6);
        redTarget2.setName("RedPerimeterTgt2");

        VuforiaTrackable frontTarget1  = skystone.get(7);
        frontTarget1.setName("FrontPerimeterTgt1");

        VuforiaTrackable frontTarget2  = skystone.get(8);
        frontTarget2.setName("FrontPerimeterTgt2");

        VuforiaTrackable blueTarget1  = skystone.get(9);
        blueTarget1.setName("BluePerimeterTgt1");

        VuforiaTrackable blueTarget2  = skystone.get(10);
        blueTarget2.setName("BluePerimeterTgt2");

        VuforiaTrackable rearTarget1  = skystone.get(11);
        rearTarget1.setName("RearPerimeterTgt1");

        VuforiaTrackable rearTarget2  = skystone.get(12);
        rearTarget2.setName("RearPerimeterTgt2");

        /** For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(skystone);

        /**
         * We use units of mm here because that's the recommended units of measurement for the
         * size values specified in the XML for the ImageTarget trackables in data sets. E.g.:
         *      <ImageTarget name="stones" size="247 173"/>
         * You don't *have to* use mm here, but the units here and the units used in the XML
         * target configuration files *must* correspond for the math to work out correctly.
         */
        float mmPerInch        = 25.4f;
        float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
        float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

        /**
         * In order for localization to work, we need to tell the system where each target we
         * wish to use for navigation resides on the field, and we need to specify where on the robot
         * the phone resides. These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * For the most part, you don't need to understand the details of the math of how transformation
         * matrices work inside (as fascinating as that is, truly). Just remember these key points:
         * <ol>
         *
         *     <li>You can put two transformations together to produce a third that combines the effect of
         *     both of them. If, for example, you have a rotation transform R and a translation transform T,
         *     then the combined transformation matrix RT which does the rotation first and then the translation
         *     is given by {@code RT = T.multiplied(R)}. That is, the transforms are multiplied in the
         *     <em>reverse</em> of the chronological order in which they applied.</li>
         *
         *     <li>A common way to create useful transforms is to use methods in the {@link OpenGLMatrix}
         *     class and the Orientation class. See, for example, {@link OpenGLMatrix#translation(float,
         *     float, float)}, {@link OpenGLMatrix#rotation(AngleUnit, float, float, float, float)}, and
         *     {@link Orientation#getRotationMatrix(AxesReference, AxesOrder, AngleUnit, float, float, float)}.
         *     Related methods in {@link OpenGLMatrix}, such as {@link OpenGLMatrix#rotated(AngleUnit,
         *     float, float, float, float)}, are syntactic shorthands for creating a new transform and
         *     then immediately multiplying the receiver by it, which can be convenient at times.</li>
         *
         *     <li>If you want to break open the black box of a transformation matrix to understand
         *     what it's doing inside, use {@link MatrixF#getTranslation()} to fetch how much the
         *     transform will move you in x, y, and z, and use {@link Orientation#getOrientation(MatrixF,
         *     AxesReference, AxesOrder, AngleUnit)} to determine the rotational motion that the transform
         *     will impart. See {@link #format(OpenGLMatrix)} below for an example.</li>
         *
         * </ol>
         *
         * This example places the "stones" image on the perimeter wall to the Left
         *  of the Red Driver station wall.  Similar to the Red Beacon Location on the Res-Q
         *
         * This example places the "chips" image on the perimeter wall to the Right
         *  of the Blue Driver station.  Similar to the Blue Beacon Location on the Res-Q
         *
         * See the doc folder of this project for a description of the field Axis conventions.
         *
         * Initially the target is conceptually lying at the origin of the field's coordinate system
         * (the center of the field), facing up.
         *
         * In this configuration, the target's coordinate system aligns with that of the field.
         *
         * In a real situation we'd also account for the vertical (Z) offset of the target,
         * but for simplicity, we ignore that here; for a real robot, you'll want to fix that.
         *
         * To place the Stones Target on the Red Audience wall:
         * - First we rotate it 90 around the field's X axis to flip it upright
         * - Then we rotate it  90 around the field's Z access to face it away from the audience.
         * - Finally, we translate it back along the X axis towards the red audience wall.
         */

        OpenGLMatrix redTarget1LocationOnField = OpenGLMatrix

                .translation(mmFTCFieldWidth/2, (mmFTCFieldWidth/2) - 914.4f, 146.05f)
                .multiplied(Orientation.getRotationMatrix(

                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        redTarget1.setLocation(redTarget1LocationOnField);
        RobotLog.ii(TAG, "Red Target 1 =%s", format(redTarget1LocationOnField));


        OpenGLMatrix redTarget2LocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth/2, (-mmFTCFieldWidth/2) + 914.4f, 146.05f)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        redTarget2.setLocation(redTarget2LocationOnField);
        RobotLog.ii(TAG, "Red Target 2 =%s", format(redTarget2LocationOnField));




        OpenGLMatrix rearTarget1LocationOnField = OpenGLMatrix

                .translation((-mmFTCFieldWidth/2) + 914.4f, (mmFTCFieldWidth/2) , 146.05f)
                .multiplied(Orientation.getRotationMatrix(

                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 180, 180, 0));
        rearTarget1.setLocation(rearTarget1LocationOnField);
        RobotLog.ii(TAG, "Rear Target 1 =%s", format(rearTarget1LocationOnField));


        OpenGLMatrix rearTarget2LocationOnField = OpenGLMatrix


                .translation((mmFTCFieldWidth/2) - 914.4f, mmFTCFieldWidth/2, 146.05f)
                .multiplied(Orientation.getRotationMatrix(

                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 180, 0, 0));
        rearTarget2.setLocation(rearTarget2LocationOnField);
        RobotLog.ii(TAG, "Rear Target 1 =%s", format(rearTarget2LocationOnField));




        OpenGLMatrix frontTarget1LocationOnField = OpenGLMatrix

                .translation((mmFTCFieldWidth/2) - 914.4f, (-mmFTCFieldWidth/2), 146.05f)
                .multiplied(Orientation.getRotationMatrix(

                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        frontTarget1.setLocation(frontTarget1LocationOnField);
        RobotLog.ii(TAG, "Front Target 1 =%s", format(frontTarget1LocationOnField));


        OpenGLMatrix frontTarget2LocationOnField = OpenGLMatrix

                .translation((-mmFTCFieldWidth/2) + 914.4f, (-mmFTCFieldWidth/2), 146.05f)
                .multiplied(Orientation.getRotationMatrix(

                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        frontTarget2.setLocation(frontTarget2LocationOnField);
        RobotLog.ii(TAG, "Front Target 1 =%s", format(frontTarget2LocationOnField));




        OpenGLMatrix blueTarget1LocationOnField = OpenGLMatrix

                .translation(-mmFTCFieldWidth/2, (-mmFTCFieldWidth/2) + 914.4f, 146.05f)
                .multiplied(Orientation.getRotationMatrix(

                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 0, 0, 90));
        blueTarget1.setLocation(blueTarget1LocationOnField);
        RobotLog.ii(TAG, "Blue Target 1 =%s", format(blueTarget1LocationOnField));


        OpenGLMatrix blueTarget2LocationOnField = OpenGLMatrix

                .translation(-mmFTCFieldWidth/2, (mmFTCFieldWidth/2) - 914.4f, 146.05f)
                .multiplied(Orientation.getRotationMatrix(

                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 0, 0, 90));
        blueTarget2.setLocation(blueTarget2LocationOnField);
        RobotLog.ii(TAG, "Blue Target 2 =%s", format(frontTarget1LocationOnField));





        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(mmBotWidth/4,0,0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, -180, 0, 0));
        RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));

        /**
         * Let the trackable listeners we care about know where the phone is. We know that each
         * listener is a {@link VuforiaTrackableDefaultListener} and can so safely cast because
         * we have not ourselves installed a listener of a different type.
         */

        ((VuforiaTrackableDefaultListener)redTarget1.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)redTarget2.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)blueTarget1.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)blueTarget2.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)frontTarget1.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)frontTarget2.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)rearTarget1.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)rearTarget2.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);



        /**
         * A brief tutorial: here's how all the math is going to work:
         *
         * C = phoneLocationOnRobot  maps   phone coords -> robot coords
         * P = tracker.getPose()     maps   image target coords -> phone coords
         * L = redTargetLocationOnField maps   image target coords -> field coords
         *
         * So
         *
         * C.inverted()              maps   robot coords -> phone coords
         * P.inverted()              maps   phone coords -> imageTarget coords
         *
         * Putting that all together,
         *
         * L x P.inverted() x C.inverted() maps robot coords to field coords.
         *
         * @see VuforiaTrackableDefaultListener#getRobotLocation()
         */

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        /** Start tracking the data sets we care about. */
        int stage = 0;
        skystone.activate();

        while (opModeIsActive()) {
            for (VuforiaTrackable trackable : allTrackables) {
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
            if(stage == 0){
                fl.setPower(1);
                bl.setPower(1);
                fr.setPower(1);
                br.setPower(1);
                if(lastLocation != null && lastLocation.get(0,0) > (mmFTCFieldWidth/4) + 100){
                    stage++;
                    //wait(500);
                }
            }

            if(stage == 1){
                fl.setPower(-1);
                bl.setPower(-1);
                fr.setPower(-1);
                br.setPower(-1);
                if(lastLocation.get(0,0) < (mmFTCFieldWidth/4)){
                    stage++;
                }
            }

            if(stage == 2){
                fl.setPower(-1);
                bl.setPower(1);
                fr.setPower(1);
                br.setPower(-1);
                if(lastLocation.get(0,1) < 0){
                    stage++;
                }
            }

            if(stage == 3){
                fl.setPower(1);
                bl.setPower(1);
                fr.setPower(1);
                br.setPower(1);
                if(lastLocation.get(0,0) < 500){
                    stage++;
                }
            }

            if(stage == 4){
                fl.setPower(1);
                bl.setPower(-1);
                fr.setPower(-1);
                br.setPower(1);
                if(lastLocation.get(0,1) > 500){
                    stage++;
                }
            }


            /**
             * Provide feedback as to where the robot was last located (if we know).
             */
            if (lastLocation != null) {
                //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
                telemetry.addData("Pos", format(lastLocation));
            } else {
                telemetry.addData("Pos", "Unknown");
            }
            telemetry.update();
        }
    }

    /**
     * A simple utility that extracts positioning information from a transformation matrix
     * and formats it in a form palatable to a human being.
     */
    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }

}
