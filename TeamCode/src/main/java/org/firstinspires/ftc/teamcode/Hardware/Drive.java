package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.math.Vector2d;


public class Drive {

    public double correction;
    public double kp;
    public double kd;

    Vector2d errorVector = new Vector2d(0,0);
    Vector2d targetVector = new Vector2d(0,0);
    Vector2d lastErrorVector = new Vector2d(0,0);
    Vector2d derivativeVector = new Vector2d(0,0);
    Vector2d correctionVector = new Vector2d(0,0);
    private double direction;
    double targetX = 0;


    //public DcMotor fl = hardwareMap.dcMotor.get("front_left_motor");
    //public DcMotor fr = hardwareMap.dcMotor.get("front_right_motor");
    //public DcMotor bl = hardwareMap.dcMotor.get("back_left_motor");
    //public DcMotor br = hardwareMap.dcMotor.get("back_right_motor");

    public void drive(Vector2d currentPositionVector, double speed){

        errorVector = Vector2d.subtract(targetVector,currentPositionVector);

        direction = Math.atan2(errorVector.y, errorVector.x);



        derivativeVector = Vector2d.subtract(errorVector,lastErrorVector);
        correctionVector = Vector2d.add(Vector2d.multiply(kp, errorVector), Vector2d.multiply(kd, derivativeVector));


        fr.setPower(speed * (correctionVector.x + correctionVector.y));
        fl.setPower(speed * (correctionVector.x - correctionVector.y));
        br.setPower(speed * (correctionVector.x - correctionVector.y));
        bl.setPower(speed * (correctionVector.x + correctionVector.y));


        lastErrorVector = errorVector;

    }
    public void setTarget(Vector2d targetVector){
        this.targetVector = targetVector;
    }
}
