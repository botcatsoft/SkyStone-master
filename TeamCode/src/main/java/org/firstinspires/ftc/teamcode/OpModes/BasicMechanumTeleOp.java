package org.firstinspires.ftc.teamcode.OpModes;

        import com.qualcomm.hardware.bosch.BNO055IMU;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.Servo;
        import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
        import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
        import org.firstinspires.ftc.teamcode.Hardware.Arm;
        import org.firstinspires.ftc.teamcode.OpModes.Abstract.BaseOpMode;
        import org.firstinspires.ftc.teamcode.math.Vector2d;


@TeleOp(name = "Mechanum")public class BasicMechanumTeleOp extends BaseOpMode {
    public void runOpMode() {
        //Variables

        boolean rightBumper = true;
        boolean leftBumper = true;
        boolean clawDebounce = false;
        boolean flippyDebounce = false;
        double armTarget;
        double error;
        double lastError = 0;
        double correction;
        double kd = 0.5;
        double kp = 0.5;
        double derivative;




        clawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Servo flipServo = hardwareMap.servo.get("flippy");
        Servo clawServo = hardwareMap.servo.get("claw_servo");

        //encoders yay


        //Motors
        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.FORWARD);

        //Gamepad
        double frPower = 0;
        double flPower = 0;
        double brPower = 0;
        double blPower = 0;

        // Gyro init

        param.loggingEnabled = false;
        param.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        param.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(param);


        clawServo.setPosition(0.82);
        flipServo.setPosition(1);
        
        waitForStart();
        Arm intake = new Arm();

        while (opModeIsActive()) {

            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            //angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);




            //toggles rightbumper variable
            if (gamepad1.right_bumper && clawDebounce == false) {
                if (rightBumper == false) {
                    rightBumper = true;
                } else {
                    rightBumper = false;
                }
                clawDebounce = true;

            } else if (gamepad1.right_bumper == false) {
                clawDebounce = false;
            }

            //moves arm
            if (rightBumper) {
                armTarget = 0; // up
                //MotorStuff
                //If Arm is Down then go up if pressed
            } else {
                armTarget = 90;  // touching mat
                //MotorStuff
            }





          error = armTarget - clawMotor.getCurrentPosition();
          derivative = error - lastError;
          correction = kp * error + kd * derivative;
          clawMotor.setPower(correction);

            //flippy variable switch
            if (gamepad1.left_bumper && flippyDebounce == false) {
                if (!leftBumper) {
                    leftBumper = true;

                } else {
                    leftBumper = false;
                }
                flippyDebounce = true;

            } else if (gamepad1.left_bumper == false) {
                flippyDebounce = false;
            }

            if (leftBumper) {
                flipServo.setPosition(1.0); //touching mat
                //MotorStuff
                //If Arm is Down then go up if pressed
            } else{
                flipServo.setPosition(0.5); // up
                //MotorStuff
            }

            Vector2d input = new Vector2d(gamepad1.left_stick_y/2, gamepad1.left_stick_x/2);
            double rot = gamepad1.right_trigger - gamepad1.left_trigger;
            fl.setPower(input.x + input.y + rot);
            fr.setPower(input.x - input.y - rot);
            bl.setPower(input.x - input.y + rot);
            br.setPower(input.x + input.y - rot);

            double armSpeed = intake.move(clawMotor.getCurrentPosition());
            clawMotor.setPower(armSpeed);
            
            


            telemetry.addData("frPower: ", frPower);
            telemetry.addData("flPower: ", flPower);
            telemetry.addData("brPower: ", brPower);
            telemetry.addData("blPower: ", blPower);
            telemetry.addData("Claw Encoder: ", clawMotor.getCurrentPosition());
            telemetry.addData("rightBumper: ", rightBumper);
            telemetry.addData("spin force: ", gamepad1.right_stick_x);
            telemetry.addData("claw Servo pos: ", clawServo.getPosition());
            telemetry.update();

            }
        }
    }
