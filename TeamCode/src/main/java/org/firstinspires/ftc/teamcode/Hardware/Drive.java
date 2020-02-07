package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.math.Vector2d;


public class Drive {


    private double kp;
    private double kd;
    private double targetAngle = 0;

    private Vector2d errorVector = new Vector2d(0,0);
    private Vector2d targetVector = new Vector2d(0,0);
    private Vector2d lastErrorVector = new Vector2d(0,0);
    private Vector2d derivativeVector = new Vector2d(0,0);
    private Vector2d correctionVector = new Vector2d(0,0);
    private Vector2d scaledErrorVector = new Vector2d(0,0);

    public Drive(){

    }



    public Vector2d drive(Vector2d currentPositionVector){
        //error is target - current
        errorVector.set(Vector2d.subtract(targetVector,currentPositionVector));

        //normalize the error vector because we choose the scale here
        scaledErrorVector.set(Vector2d.multiply(errorVector, 0.0005));

        //sets derivative and correction vectors
        derivativeVector.set(Vector2d.subtract(scaledErrorVector,lastErrorVector));
        correctionVector.set(Vector2d.add(Vector2d.multiply(kp, scaledErrorVector), Vector2d.multiply(kd, derivativeVector)));


        //last error recursivity
        lastErrorVector.set(scaledErrorVector);
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

    public double getErrorX(Vector2d currentPositionVector){
        errorVector.set(Vector2d.subtract(targetVector,currentPositionVector));
        return errorVector.x;
    }

    public double getErrorY(Vector2d currentPositionVector){
        errorVector.set(Vector2d.subtract(targetVector,currentPositionVector));
        return errorVector.y;
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
