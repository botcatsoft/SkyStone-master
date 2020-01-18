package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Hardware.Drive;

@TeleOp(name = "Kp and Kd Setter") public class KpKdSetter extends LinearOpMode {
    public void runOpMode(){

        DcMotor fl = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor fr = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor bl = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor br = hardwareMap.dcMotor.get("back_right_motor");
        DcMotor clawMotor = hardwareMap.dcMotor.get("claw_motor");
        double error;
        double lastError = 0;

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters param = new BNO055IMU.Parameters();

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