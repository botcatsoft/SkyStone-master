package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.OpModes.Abstract.BaseOpMode;

@TeleOp(name = "servo test") public class JavaMonsterServoTest extends BaseOpMode {
    public void runOpMode(){
        Servo servoTest = hardwareMap.servo.get("servo_test");



        waitForStart();
        while(opModeIsActive()){


            if(gamepad1.y) {
                // move to 0 degrees.
                servoTest.setPosition(0);
            } else if (gamepad1.x || gamepad1.b) {
                // move to 90 degrees.
                servoTest.setPosition(0.5);
            } else if (gamepad1.a) {
                // move to 180 degrees.
                servoTest.setPosition(1);
            }
            telemetry.addData("Servo Position", servoTest.getPosition());


            telemetry.update();

        }
        }
    }