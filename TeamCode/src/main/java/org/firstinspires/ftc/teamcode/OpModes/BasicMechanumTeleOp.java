package org.firstinspires.ftc.teamcode.OpModes;

        import com.qualcomm.hardware.bosch.BNO055IMU;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.CRServo;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.Servo;

        import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
        import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
        import org.firstinspires.ftc.teamcode.Hardware.Arm;
        import org.firstinspires.ftc.teamcode.math.Vector2d;


@TeleOp(name = "Mechanum")public class BasicMechanumTeleOp extends LinearOpMode {
    public void runOpMode() {
        //Variables


        boolean rightArmB = true;
        boolean BaseGrabberDebounce = false;


        DcMotor fl = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor fr = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor bl = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor br = hardwareMap.dcMotor.get("back_right_motor");
        DcMotor clawMotor = hardwareMap.dcMotor.get("claw_motor");
        DcMotor handMotor = hardwareMap.dcMotor.get("hand_motor");


        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters param = new BNO055IMU.Parameters();


        clawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        CRServo handServo = hardwareMap.crservo.get("hand_servo");
        Servo buildPlateServo = hardwareMap.servo.get("BuildPlate_servo");
        Servo buildPlateServo2 = hardwareMap.servo.get("Build_Plate_servo");
        CRServo liftServoR = hardwareMap.crservo.get("right_lift_servo");
        CRServo liftServoL = hardwareMap.crservo.get("left_lift_servo");
        //Servo doorServo = hardwareMap.servo.get("Door_servo");

        buildPlateServo.setPosition(0.4);



        //Motors
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

        // Gyro init

        param.loggingEnabled = false;
        param.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        param.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(param);
        buildPlateServo.setPosition(0.4);


        waitForStart();

        double armSpeed = 10;
        double currentX = 0;
        double currentY = 0;
        double initialAngle = 0;
        double angle;
        double lastfr = 0;
        double lastfl = 0;
        double lastbr = 0;
        double lastbl = 0;
        double goRightMotors;
        double goLeftMotors;
        double dx;
        double dy;

        while (opModeIsActive()) {
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);




            //hand control
            if (gamepad2.dpad_down) {
                liftServoR.setPower(-0.3);
                liftServoL.setPower(0.3);
                //doorServo.setPosition(0.35);
            } else if (gamepad2.dpad_up) {
                liftServoR.setPower(0.3);
                liftServoL.setPower(-0.3);
                //doorServo.setPosition(0);
            } else {
                liftServoR.setPower(0);
                liftServoL.setPower(0);
            }

            if (gamepad2.right_bumper) {
                handServo.setPower(-0.3);
                //doorServo.setPosition(0.35);
            } else if (gamepad2.left_bumper) {
                handServo.setPower(0.3);
                //doorServo.setPosition(0);
            } else {
                handServo.setPower(0);
            }


            if(gamepad1.a){
                armSpeed = 10;
            }else if(gamepad1.b){
                armSpeed = 40;
            }


            //motor setting for drivetrain
            Vector2d input = new Vector2d(gamepad1.left_stick_y / 2, gamepad1.left_stick_x / 2);
            double rot = gamepad1.right_trigger - gamepad1.left_trigger;
            fl.setPower(input.x - input.y - rot);
            fr.setPower(input.x + input.y + rot);
            bl.setPower(input.x + input.y - rot);
            br.setPower(input.x - input.y + rot);

            //motor setting for arm
            //double armSpeed = intake.move(clawMotor.getCurrentPosition());
            //clawMotor.setPower(armSpeed);
            clawMotor.setPower((gamepad2.right_stick_y / armSpeed));

            handMotor.setPower((gamepad2.left_stick_y / 10));

            //toggles Build Plate Grabbers
            if (gamepad1.x && !BaseGrabberDebounce) {
                rightArmB = !rightArmB;
                BaseGrabberDebounce = true;

            } else if (!gamepad1.x) {
                BaseGrabberDebounce = false;
            }

            if (rightArmB) {
                buildPlateServo.setPosition(0.4);
                buildPlateServo2.setPosition(1);

            } else {
                buildPlateServo.setPosition(1);
                buildPlateServo2.setPosition(0.6);
            }





            goRightMotors = (fl.getCurrentPosition() - lastfl + br.getCurrentPosition() - lastbr) / 2;
            goLeftMotors = (fr.getCurrentPosition() - lastfr + bl.getCurrentPosition() - lastbl) / 2;

            dx = goRightMotors - goLeftMotors;
            dy = goRightMotors + goLeftMotors;

            angle = angles.firstAngle + initialAngle;
            currentX = currentX + (Math.cos(angle) * dx + Math.sin(angle) * dy);
            currentY = currentY + (Math.cos(angle) * dy + Math.sin(angle) * dx);

            lastfl = fl.getCurrentPosition();
            lastfr = fr.getCurrentPosition();
            lastbl = bl.getCurrentPosition();
            lastbr = br.getCurrentPosition();


            /*telemetry.addData("frPower: ", fr.getPower());
            telemetry.addData("flPower: ", fl.getPower());
            telemetry.addData("brPower: ", br.getPower());
            telemetry.addData("blPower: ", bl.getPower());
            telemetry.addData("x: ", gamepad1.x);
            telemetry.addData("Build Plate Servo", buildPlateServo.getPosition());
            telemetry.addData("Build Plate Servo2", buildPlateServo2.getPosition());
            telemetry.addData("Right Bumper: ", gamepad1.right_bumper);
            telemetry.addData("Left Bumper: ", gamepad1.left_bumper);
            telemetry.addData("Hand Power: ", handServo.getPower());
            telemetry.addData("Claw Encoder: ", clawMotor.getCurrentPosition());
            telemetry.addData("Claw Power", armSpeed);*/
            telemetry.addData("Xpos", currentX);
            telemetry.addData("Ypos", currentY);
            telemetry.addData("Angle", angle);
            //telemetry.addData("Door Location", doorServo.getPosition());

            telemetry.update();


        }
    }
}
