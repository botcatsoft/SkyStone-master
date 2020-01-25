package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.math.Vector2d;


public class Drive {


    public double kp;
    public double kd;
    public double targetAngle = 0;

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


        //last error recursivity
        lastErrorVector = normalizedErrorVector;
        return correctionVector;

    }

//target setter
    public void setTarget(double angle){
        targetAngle = angle;
    }
    public void setTarget(double x, double y){

        targetVector.set(x,y);

    }

    public void setTarget(double x, double y, double angle){

        targetVector.set(x,y);
        targetAngle = angle;
    }

    public double getAngle(){
        return targetAngle;
    }

    public double getkp(){
        return kp;
    }

    public double getkd(){
        return kd;
    }

    public void setkp(double newKp) { kp = newKp; }

    public void setkd(double newKd) { kd = newKd; }
    
    public boolean atTarget() {
        return (errorVector.getMagnitude() < 10);

    }
}
