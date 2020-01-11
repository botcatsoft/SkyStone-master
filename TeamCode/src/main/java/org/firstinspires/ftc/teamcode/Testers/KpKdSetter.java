package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Hardware.Drive;
import org.firstinspires.ftc.teamcode.OpModes.Abstract.BaseOpMode;

@TeleOp(name = "Kp and Kd Setter") public class KpKdSetter extends BaseOpMode {
    public void runOpMode(){
        double error;
        double lastError = 0;

        waitForStart();
        Drive setterDrive = new Drive();
        boolean debounce  = true;
        while(opModeIsActive()){
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            error = setterDrive.getAngle() - angles.firstAngle;
            double derivative = error - lastError;
            double correction = setterDrive.getkp() * error + setterDrive.getkd() * derivative;

            if( gamepad1.a){
                setterDrive.setkp(setterDrive.getkp() + 0.01);

            }
            if( gamepad1.b){
                setterDrive.setkd(setterDrive.getkd() + 0.01);
            }

            if( gamepad1.x){
                setterDrive.setkp(setterDrive.getkp() - 0.01);
            }

            if( gamepad1.y){
                setterDrive.setkd(setterDrive.getkd() - 0.01);
            }

            if(gamepad1.right_bumper && debounce){
                setterDrive.setTarget((double)angles.firstAngle + 90);
                debounce = false;
            }
            else if(gamepad1.right_bumper == false){
                debounce = true;
            }

            fr.setPower(-correction);
            br.setPower(-correction);
            fl.setPower(correction);
            bl.setPower(correction);
            lastError = error;

            telemetry.addData("Kp: ", setterDrive.getkp());
            telemetry.addData("Kd: ", setterDrive.getkd());
            telemetry.update();
        }
    }
}