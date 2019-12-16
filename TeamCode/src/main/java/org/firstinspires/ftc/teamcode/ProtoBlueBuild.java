package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "ProtoBlueBuild") public class ProtoBlueBuild extends LinearOpMode {
    @Override public void runOpMode() {
      //Variables
      int stage = 0;
      //Get Motors
        DcMotor fl = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor fr = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor bl = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor br = hardwareMap.dcMotor.get("back_right_motor");
        Servo flipServo = hardwareMap.servo.get("flippy");
        Servo clawServo = hardwareMap.servo.get("claw_servo");

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        clawServo.setPosition(0.82);
        flipServo.setPosition(1);

        waitForStart();
        while(opModeIsActive()){
          //Go Forward
            if(stage == 0){
                fl.setPower(1);
                bl.setPower(1);
                fr.setPower(1);
                br.setPower(1);
                if(fl.getCurrentPosition() > 1200){
                    stage++;
                }
            }
            //Set Claw Down
            if(stage == 1){
                clawServo.setPosition(0);
                stage++;
            }
            //Drive BackWards
            if(stage == 2){
                fl.setPower(-1);
                bl.setPower(-1);
                fr.setPower(-1);
                br.setPower(-1);
                if(fl.getCurrentPosition() < 5){
                    stage++;
                }
            }
            //Set Claw Up
            if(stage == 3){
                clawServo.setPosition(0.82);
                stage++;
            }
            //Go Right
            if(stage == 4){
                fl.setPower(1);
                bl.setPower(-1);
                fr.setPower(-1);
                br.setPower(1);
                if(fl.getCurrentPosition() > 900){
                    stage++;
                }
            }
           //Go Forward
            if(stage == 5){
              fl.setPower(1);
              bl.setPower(1);
              fr.setPower(1);
              br.setPower(1);
              if(fl.getCurrentPosition() > 1400){
                stage++;
              }
            }
            //Go Left
            if(stage == 6){
                fl.setPower(-1);
                bl.setPower(1);
                fr.setPower(1);
                br.setPower(-1);
                if(fl.getCurrentPosition() < 500){
                    stage++;
                }
            }
            //Drive BackWards
            if(stage == 7){
                fl.setPower(-1);
                bl.setPower(-1);
                fr.setPower(-1);
                br.setPower(-1);
                if(fl.getCurrentPosition() < 200){
                    stage++;
                }
            }
       }
    }
}
