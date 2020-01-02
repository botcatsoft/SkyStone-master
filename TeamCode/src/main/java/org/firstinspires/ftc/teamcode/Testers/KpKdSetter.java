package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Drive;
import org.firstinspires.ftc.teamcode.OpModes.Abstract.BaseOpMode;

@TeleOp(name = "Kp and Kd Setter") public class KpKdSetter extends BaseOpMode {
    public void runOpMode(){

        waitForStart();
        Drive setterDrive = new Drive();
        while(opModeIsActive()){

            if( gamepad1.a == true ){
                setterDrive.setkp(setterDrive.getkp() + 0.01);
            }

            if( gamepad1.b == true ){
                setterDrive.setkd(setterDrive.getkd() + 0.01);
            }

            if( gamepad1.x == true ){
                setterDrive.setkp(setterDrive.getkp() - 0.01);
            }

            if( gamepad1.y == true ){
                setterDrive.setkd(setterDrive.getkd() - 0.01);
            }

            telemetry.addData("Kp: ", setterDrive.getkp());
            telemetry.addData("Kd: ", setterDrive.getkd());
            telemetry.update();
        }

    }
}
