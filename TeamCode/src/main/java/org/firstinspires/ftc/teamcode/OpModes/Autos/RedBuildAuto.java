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
import org.firstinspires.ftc.teamcode.Hardware.Drive;
import org.firstinspires.ftc.teamcode.OpModes.Abstract.BaseOpMode;
import org.firstinspires.ftc.teamcode.math.Vector2d;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="RedBuildAuto", group ="Concept")
public class RedBuildAuto extends BaseOpMode {
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


        parameters.vuforiaLicenseKey = "AVop5Pr/////AAABmYh62mTDYUgSsC1tOVg8A0IjuyqMzLO1I9Ue7E0egk11CAzRZeX8GW2ytNBbBI96DbCQ306sVAjACJXwPhaiHFG5dHPRk3daxk2TPpzhXHEinXui59HAdySDhWwyxKSyBc7tkFk7cR2JOThtf5Qy7CJgp3qa8MxpZ2ZhCt433Iji1nMtL69cC9QVA260v7XJkWcXrGomJQYgrZl3w4AkQNrzkUSejo7LBHcaGyy2SHEMppAt8KXvSlDYJJgm737AVy27wgdGoMcS45F5VrAHhaUxy1os4ZDxfm2eNCFyCfzUc+M8jsewh2RZvk98rIbpGyAv7ZVqppF+G9HqQThn+lEGNm0Jy0nEHYajG0pHxVI4";


        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        /**
         * Instantiate the Vuforia engine
         */
        vuforia = ClassFactory.getInstance().createVuforia(parameters);


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

        VuforiaTrackable bridgeBlueRear  = skystone.get(2);
        bridgeBlueRear.setName("BridgeBlueRear");

        VuforiaTrackable bridgeRedRear  = skystone.get(3);
        bridgeRedRear.setName("BridgeRedRear");

        VuforiaTrackable bridgeBlueFront  = skystone.get(4);
        bridgeBlueFront.setName("BridgeBlueFront");

        VuforiaTrackable bridgeRedFront  = skystone.get(5);
        bridgeRedFront.setName("BridgeRedFront");

        /** For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(skystone);


        float mmPerInch        = 25.4f;
        float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
        float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels


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

        OpenGLMatrix bridgeBlueRearLocationOnField = OpenGLMatrix

                .translation(-mmFTCFieldWidth/2, (mmFTCFieldWidth/2) - 914.4f, 146.05f)
                .multiplied(Orientation.getRotationMatrix(

                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 180, -60, 0));
        blueTarget2.setLocation(bridgeBlueRearLocationOnField);
        RobotLog.ii(TAG, "Bridge Blue Rear =%s", format(bridgeBlueRearLocationOnField));

        OpenGLMatrix bridgeRedRearLocationOnField = OpenGLMatrix

                .translation(-mmFTCFieldWidth/2, (mmFTCFieldWidth/2) - 914.4f, 146.05f)
                .multiplied(Orientation.getRotationMatrix(

                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 0, 60, 0));
        blueTarget2.setLocation(bridgeRedRearLocationOnField);
        RobotLog.ii(TAG, "Bridge Red Rear =%s", format(bridgeRedRearLocationOnField));

        OpenGLMatrix bridgeBlueFrontLocationOnField = OpenGLMatrix

                .translation(-mmFTCFieldWidth/2, (mmFTCFieldWidth/2) - 914.4f, 146.05f)
                .multiplied(Orientation.getRotationMatrix(

                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 180, -60, 0));
        blueTarget2.setLocation(bridgeBlueFrontLocationOnField);
        RobotLog.ii(TAG, "Bridge Blue Front =%s", format(bridgeBlueFrontLocationOnField));

        OpenGLMatrix bridgeRedFrontLocationOnField = OpenGLMatrix

                .translation(-mmFTCFieldWidth/2, (mmFTCFieldWidth/2) - 914.4f, 146.05f)
                .multiplied(Orientation.getRotationMatrix(

                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 0, 60, 0));
        blueTarget2.setLocation(bridgeRedFrontLocationOnField);
        RobotLog.ii(TAG, "Bridge Red Front =%s", format(bridgeRedFrontLocationOnField));

        OpenGLMatrix bridgeBlueRearLocationOnField = OpenGLMatrix

                .translation(-mmFTCFieldWidth/2, (mmFTCFieldWidth/2) - 914.4f, 146.05f)
                .multiplied(Orientation.getRotationMatrix(

                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 180, -60, 0));
        blueTarget2.setLocation(bridgeBlueRearLocationOnField);
        RobotLog.ii(TAG, "Bridge Blue Rear =%s", format(bridgeBlueRearLocationOnField));

        OpenGLMatrix bridgeRedRearLocationOnField = OpenGLMatrix

                .translation(-mmFTCFieldWidth/2, (mmFTCFieldWidth/2) - 914.4f, 146.05f)
                .multiplied(Orientation.getRotationMatrix(

                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 0, 60, 0));
        blueTarget2.setLocation(bridgeRedRearLocationOnField);
        RobotLog.ii(TAG, "Bridge Red Rear =%s", format(bridgeRedRearLocationOnField));

        OpenGLMatrix bridgeBlueFrontLocationOnField = OpenGLMatrix

                .translation(-mmFTCFieldWidth/2, (mmFTCFieldWidth/2) - 914.4f, 146.05f)
                .multiplied(Orientation.getRotationMatrix(

                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 180, -60, 0));
        blueTarget2.setLocation(bridgeBlueFrontLocationOnField);
        RobotLog.ii(TAG, "Bridge Blue Front =%s", format(bridgeBlueFrontLocationOnField));

        OpenGLMatrix bridgeRedFrontLocationOnField = OpenGLMatrix

                .translation(-mmFTCFieldWidth/2, (mmFTCFieldWidth/2) - 914.4f, 146.05f)
                .multiplied(Orientation.getRotationMatrix(

                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 0, 60, 0));
        blueTarget2.setLocation(bridgeRedFrontLocationOnField);
        RobotLog.ii(TAG, "Bridge Red Front =%s", format(bridgeRedFrontLocationOnField));

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(mmBotWidth/4,0,0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, -180, 0, 0));
        RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));

        //Wall Target listeners
        ((VuforiaTrackableDefaultListener)redTarget1.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)redTarget2.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)blueTarget1.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)blueTarget2.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)frontTarget1.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)frontTarget2.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)rearTarget1.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)rearTarget2.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

        //listeners for bridge trackables
        ((VuforiaTrackableDefaultListener)bridgeRedRear.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)bridgeBlueRear.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)bridgeRedFront.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)bridgeBlueFront.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);



        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        /** Start tracking the data sets we care about. */
        int stage = 0;
        Drive homer = new Drive();

        skystone.activate();

        while (opModeIsActive()) {
            for (VuforiaTrackable trackable : allTrackables) {

                telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }

            }

            //drive up to site
            if(stage == 0){
                homer.setTarget(596.9,1193.8);
            }

            //latch on
            if(stage == 1){
              //latch on
            }

            //pull site back
            if(stage == 2){
                homer.setTarget(1562.1,1193.8);
            }

            //unlatch
            if(stage == 3){
               //unlatch
            }

            //drive towards bridge past site
            if(stage == 4){
                homer.setTarget(1562.1, 300);
            }

            //drive towards blue side past site
            if(stage == 5){
                homer.setTarget(650, 300);
            }

            //drive away from bridge so we can push site back
            if(stage == 6){
                homer.setTarget(650, 1193.8);
            }

            //push site back against the wall
            if(stage == 7){
                homer.setTarget(1200, 1193.8);
            }

            //drive under bridge for big points
            if(stage > 7){
                homer.setTarget(1200, 0);
            }

            if(homer.atTarget()){
                stage++;
            }

            Vector2d correction;
            Vector2d currentPosition = new Vector2d(lastLocation.get(0,0), lastLocation.get(0,1));
            correction = homer.drive(currentPosition, 1);

            fr.setPower(-correction.x + correction.y);
            fl.setPower(-correction.x - correction.y);
            br.setPower(-correction.x - correction.y);
            bl.setPower(-correction.x + correction.y);




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
