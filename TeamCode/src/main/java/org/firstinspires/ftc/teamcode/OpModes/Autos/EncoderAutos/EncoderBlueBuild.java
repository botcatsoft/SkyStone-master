package org.firstinspires.ftc.teamcode.OpModes.Autos.EncoderAutos;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Hardware.Arm;
import org.firstinspires.ftc.teamcode.Hardware.Drive;
import org.firstinspires.ftc.teamcode.math.Vector2d;

import static org.firstinspires.ftc.teamcode.math.Vector2d.rotate;

@Autonomous(name="EncoderBlueBuild", group ="Concept")
public class EncoderBlueBuild extends LinearOpMode {
    public void runOpMode(){
        DcMotor fl = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor fr = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor bl = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor br = hardwareMap.dcMotor.get("back_right_motor");
        DcMotor clawMotor = hardwareMap.dcMotor.get("claw_motor");

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
        clawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        CRServo handServo = hardwareMap.crservo.get("hand_servo");
        Servo buildPlateServo = hardwareMap.servo.get("BuildPlate_servo");
        Servo buildPlateServo2 = hardwareMap.servo.get("Build_Plate_servo");

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters param = new BNO055IMU.Parameters();

        param.loggingEnabled = false;
        param.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        param.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(param);


        waitForStart();

        /** Start tracking the data sets we care about. */
        int stage = 0;
        Drive homer = new Drive();
        Arm intake = new Arm();
        int loop = 0;
        double currentX = -1770;
        double currentY = 696.38;
        double initialAngle = 90;
        double angle;
        double lastfr = 0;
        double lastfl = 0;
        double lastbr = 0;
        double lastbl = 0;
        double goRightMotors;
        double goLeftMotors;
        double dx;
        double dy;

        while(opModeIsActive()) {
           Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            if(stage == 0){
                homer.setTarget(-825.5, 1193.8);
            }

            //latch on
            if(stage == 1){
                buildPlateServo.setPosition(1);
                buildPlateServo2.setPosition(0.6);
            }

            //pull site back
            if(stage == 2){
                homer.setTarget(-1562.1,1193.8);
            }

            //unlatch
            if(stage == 3){
              buildPlateServo.setPosition(0.4);
              buildPlateServo2.setPosition(1);
            }

            //drive towards bridge past site
            if(stage == 4){
              homer.setTarget(-1562.1, 300);
            }

            //drive towards blue side past site
            if(stage == 5){
              homer.setTarget(-650, 300);
            }

            //drive away from bridge so we can push site back
            if(stage == 6){
              homer.setTarget(-650, 1193.8);
            }

            //push site back against the wall
            if(stage == 7){
              homer.setTarget(-1200, 1193.8);
            }

            //drive under bridge for big points
            if(stage > 8){
              homer.setTarget(-1200, 0);
            }

            if(homer.atTarget()){
              stage++;
            }

          Vector2d correction;
          Vector2d currentPosition = new Vector2d(currentX, currentY);
          correction = homer.drive(currentPosition, 1);
          Vector2d correctionWithAngle = rotate(correction, angles.firstAngle);
          double rot = homer.getAngle() - angles.firstAngle;


          fl.setPower(correctionWithAngle.x - correctionWithAngle.y - rot);
          fr.setPower(correctionWithAngle.x + correctionWithAngle.y + rot);
          bl.setPower(correctionWithAngle.x + correctionWithAngle.y - rot);
          br.setPower(correctionWithAngle.x - correctionWithAngle.y + rot);

          double armSpeed = intake.move(clawMotor.getCurrentPosition());
          clawMotor.setPower(armSpeed);

          goRightMotors = (fl.getCurrentPosition() - lastfl + br.getCurrentPosition() - lastbr) / 2;
          goLeftMotors = (fr.getCurrentPosition() - lastfr + bl.getCurrentPosition() - lastbl) / 2;

          dx = goRightMotors - goLeftMotors;
          dy = goRightMotors + goLeftMotors;


          angle = angles.firstAngle + initialAngle;
          currentX = currentX + Math.cos(angle) * dx + Math.sin(angle) * dy; // use tan2 to find out how much of dx and dy to use
          currentY = currentY + Math.cos(angle) * dx + Math.sin(angle) * dy;




          //recursion on last encoder values
          lastfr = fr.getCurrentPosition();
          lastfl = fl.getCurrentPosition();
          lastbr = br.getCurrentPosition();
          lastbl = bl.getCurrentPosition();

          telemetry.addData("Xpos", currentX);
          telemetry.addData("Ypos", currentY);
          telemetry.addData("flPower: ", fl.getPower());
          telemetry.addData("frPower: ", fr.getPower());
          telemetry.addData("blPower: ", bl.getPower());
          telemetry.addData("brPower: ", br.getPower());
          telemetry.addData("Xpos", stage);
          telemetry.update();

        }
    }
}
