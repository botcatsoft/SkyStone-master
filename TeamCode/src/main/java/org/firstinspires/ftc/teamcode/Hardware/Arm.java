package org.firstinspires.ftc.teamcode.Hardware;

public class Arm {

    public double kp;
    public double kd;
    public double target;
    public double error;
    public double lastError = 0;
    public double derivative;
    public double correction;

    public Arm(){

    }

    public double move(double encoderValue){
        error = (target - encoderValue) / 360;
        derivative = error - lastError;
        correction = kp * error + kd * derivative;
        lastError = error;
        return correction;
    }

    public void setTarget(double targetEncoderValue){
        target = targetEncoderValue;
    }

    public boolean atTarget() {
        return (error < 0.01);
    }
}
