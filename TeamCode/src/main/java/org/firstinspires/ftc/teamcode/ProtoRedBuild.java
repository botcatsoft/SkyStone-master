package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "ProtoRedBuild") public class ProtoRedBuild extends LinearOpMode {
    @Override public void runOpMode() {
        DcMotor fl = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor fr = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor bl = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor br = hardwareMap.dcMotor.get("back_right_motor");
        Servo flipServo = hardwareMap.servo.get("flippy");
        Servo clawServo = hardwareMap.servo.get("claw_servo");

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int stage =0;

        clawServo.setPosition(0.82);
        flipServo.setPosition(1);

        waitForStart();
        while(opModeIsActive()){
          if(stage == 0){
                fl.setPower(1);
                bl.setPower(1);
                fr.setPower(1);
                br.setPower(1);
                if(fl.getCurrentPosition() >1200){
                    stage++;
                }
            }
            //move forward
          if(stage == 1){
                clawServo.setPosition(0);
                if(fl.getCurrentPosition() > 1200){

                    stage++;
                }
            }
            //drops the claw
            if(stage == 2){
                fl.setPower(-1);
                bl.setPower(-1);
                fr.setPower(-1);
                br.setPower(-1);
                if(fl.getCurrentPosition() > 1200){
                    stage++;
                }
            }
            //moves back
            if(stage == 3){
                clawServo.setPosition(0);

                stage++;

            }
            //lift up the claw
             if(stage == 4){
                fl.setPower(1);
                bl.setPower(-1);
                fr.setPower(-1);
                br.setPower(1);
                if(fl.getCurrentPosition() > 1200){
                    stage++;
                }
            }
            //moves left
          if(stage == 5){
                fl.setPower(1);
                bl.setPower(1);
                fr.setPower(1);
                br.setPower(1);
                if(fl.getCurrentPosition() >1200){
                    stage++;
                }
            }
            //moves forward
            if(stage == 6){
                fl.setPower(-1);
                bl.setPower(1);
                fr.setPower(1);
                br.setPower(-1);
                if(fl.getCurrentPosition() > 1200){
                    stage++;
                }
            }
            //moves right
            if(stage == 7){
                fl.setPower(-1);
                bl.setPower(-1);
                fr.setPower(-1);
                br.setPower(-1);
                if(fl.getCurrentPosition() >1200){
                    stage++;
                }
            }
            //moves backward
            if(stage == 8){
                fl.setPower(1);
                bl.setPower(-1);
                fr.setPower(-1);
                br.setPower(1);
                if(fl.getCurrentPosition() > 1200){
                    stage++;
                }
            }
            telemetry.addData("frPower: ", fr.getPower());
            telemetry.addData("flPower: ", fl.getPower());
            telemetry.addData("brPower: ", br.getPower());
            telemetry.addData("blPower: ", bl.getPower());

            //moves left

       }
    }
}
