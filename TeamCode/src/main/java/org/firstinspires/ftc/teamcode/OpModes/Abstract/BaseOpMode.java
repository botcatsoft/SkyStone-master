package org.firstinspires.ftc.teamcode.OpModes.Abstract;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class BaseOpMode extends LinearOpMode {

        //initialize motors
        public DcMotor fl = hardwareMap.dcMotor.get("front_left_motor");
        public DcMotor fr = hardwareMap.dcMotor.get("front_right_motor");
        public DcMotor bl = hardwareMap.dcMotor.get("back_left_motor");
        public DcMotor br = hardwareMap.dcMotor.get("back_right_motor");
        public DcMotor clawMotor = hardwareMap.dcMotor.get("claw_motor");
        //Motor Directions

        // Gyro init
        public BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        public BNO055IMU.Parameters param = new BNO055IMU.Parameters();



}
