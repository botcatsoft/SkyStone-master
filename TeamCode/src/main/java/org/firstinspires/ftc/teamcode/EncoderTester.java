package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "encoder test") public class EncoderTester extends LinearOpMode {
    public void runOpMode(){
        DcMotor clawMotor = hardwareMap.dcMotor.get("claw_motor");
        DcMotor clawMotor2 = hardwareMap.dcMotor.get("motor1");

        clawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        clawMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("encoder cm: ", clawMotor.getCurrentPosition());
            telemetry.addData("encoder m1: ", clawMotor2.getCurrentPosition());
            telemetry.update();
        }
    }
}
