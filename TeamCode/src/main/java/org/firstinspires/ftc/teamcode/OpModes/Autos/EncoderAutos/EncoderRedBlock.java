package org.firstinspires.ftc.teamcode.OpModes.Autos.EncoderAutos;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.Arm;
import org.firstinspires.ftc.teamcode.Hardware.Drive;
import org.firstinspires.ftc.teamcode.math.Vector2d;

import static org.firstinspires.ftc.teamcode.math.Vector2d.rotate;

@Autonomous(name="EncoderBlueBlock")public class EncoderRedBlock extends LinearOpMode{
      public void runOpMode(){
        DcMotor fl = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor fr = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor bl = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor br = hardwareMap.dcMotor.get("back_right_motor");
        DcMotor clawMotor = hardwareMap.dcMotor.get("claw_motor");

        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);
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
        waitForStart();

        int stage = 0;
        Drive homer = new Drive();
        Arm intake = new Arm();
        int loop = 0;
        double currentX = 1770;
        double currentY = 696.38;

        while(opModeIsActive()){
            if (stage == 0) {
              homer.setTarget(825.5, -696.38 - (loop * 198.97), 90);
              //go to block
            }
            if (stage == 1) {
              intake.setTarget(90);
              //grab_block
            }

            if (stage == 2) {
              homer.setTarget(1422.4, -696.38 - (loop * 198.97));
            } //back up

            if (stage == 3) {
              homer.setTarget(1422.4, 1193.8);
            } //move to build site

            if (stage == 4) {
              intake.setTarget(0);
              //let go of block
            }

            if (stage == 5) {
              homer.setTarget(1422.4, -696.38);
              //return to alliance side
            }

            if (stage == 6) {
              stage = 0;
              loop++;
            }

            if(homer.atTarget() && intake.atTarget()){
              stage++;
            }


        Vector2d correction;
            Vector2d currentPosition = new Vector2d(currentX, currentY);
            correction = homer.drive(currentPosition, 1);
            //Vector2d correctionWithAngle = rotate(correction, lastLocation.get(1, 2));
          }
      }
  }

