package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Mapping Test") public class MappingTest extends LinearOpMode {
    public void runOpMode(){
        DcMotor fl = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor fr = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor bl = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor br = hardwareMap.dcMotor.get("back_right_motor");

        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.FORWARD);
        waitForStart();

                waitForStart();
        while(opModeIsActive()){

            if(gamepad1.y) {
              //set motor value to 1
              fl.setPower(1);
              fr.setPower(0);
              bl.setPower(0);
              br.setPower(0);
            } else if (gamepad1.x) {
              //set motor value to 1
              fl.setPower(0);
              fr.setPower(1);
              bl.setPower(0);
              br.setPower(0);
            } else if (gamepad1.a) {
              //set motor value to 1
              fl.setPower(0);
              fr.setPower(0);
              bl.setPower(1);
              br.setPower(0);
            } else if (gamepad1.b) {
              //set motor value to 1
              fl.setPower(0);
              fr.setPower(0);
              bl.setPower(0);
              br.setPower(1);
            }
        }
    }
}
