package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.math.Vector2d;


public class Drive {


    public double kp;
    public double kd;

    Vector2d errorVector = new Vector2d(0,0);
    Vector2d targetVector = new Vector2d(0,0);
    Vector2d lastErrorVector = new Vector2d(0,0);
    Vector2d derivativeVector = new Vector2d(0,0);
    Vector2d correctionVector = new Vector2d(0,0);
    Vector2d normalizedErrorVector = new Vector2d(0,0);

    public Drive(){

    }



    public Vector2d drive(Vector2d currentPositionVector, double speed){
        //error is target - current
        errorVector = Vector2d.subtract(targetVector,currentPositionVector);
        //normalize the error vector because we choose the scale here
        normalizedErrorVector = Vector2d.multiply(errorVector, 0.001);
        //sets derivative and correction vectors
        derivativeVector = Vector2d.subtract(normalizedErrorVector,lastErrorVector);
        correctionVector = Vector2d.add(Vector2d.multiply(kp, normalizedErrorVector), Vector2d.multiply(kd, derivativeVector));

        //power setters
        //fr.setPower(speed * (correctionVector.x + correctionVector.y));
        //fl.setPower(speed * (- correctionVector.x + correctionVector.y));
        //br.setPower(speed * (- correctionVector.x + correctionVector.y));
        //bl.setPower(speed * (correctionVector.x + correctionVector.y));

        //last error recursivity
        lastErrorVector = normalizedErrorVector;
        return correctionVector;

        }

//target setter
    public void setTarget(double x, double y){

        targetVector.x = x;
        targetVector.y = y;
    }
    public boolean atTarget() {
        return (errorVector.getMagnitude() < 10);

    }
}
