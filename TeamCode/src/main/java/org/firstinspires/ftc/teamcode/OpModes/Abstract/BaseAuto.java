package org.firstinspires.ftc.teamcode.OpModes.Abstract;

public abstract class BaseAuto extends BaseOpMode {
    public double targetX;
    public double targetY;
    public double speed;
    private double direction;

    public double correction;
    public double kp;
    public double kd;
    public double error;
    public double derivative;
    public double lastError = 0;
    public double xError;
    public double yError; 

    public void drive(double X, double Y, double targetX, double targetY, double speed){
        xError = targetX - X;
        yError = targetY - Y;

        direction = Math.atan2(yError, xError);

        error = Math.sqrt(Math.pow(xError, 2) + Math.pow(yError, 2));
        derivative = error - lastError;
        correction = kp * error + kd * derivative;


        fr.setPower(speed * correction * (Math.sin(direction) + Math.cos(direction)));
        fl.setPower(speed * correction * (Math.sin(direction) - Math.cos(direction)));
        br.setPower(speed * correction * (Math.sin(direction) - Math.cos(direction)));
        bl.setPower(speed * correction * (Math.sin(direction) + Math.cos(direction)));

        lastError = error;

    }
}
