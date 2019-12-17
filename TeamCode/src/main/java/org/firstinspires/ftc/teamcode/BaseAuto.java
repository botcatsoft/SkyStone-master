package org.firstinspires.ftc.teamcode;

public abstract class BaseAuto extends BaseOpMode {
    public double targetX;
    public double targetY;
    public double speed;
    private double direction;
    public void drive(double X, double Y, double targetX, double targetY, double speed){
        direction = Math.atan2(targetY - Y, targetX - X);
        fr.setPower(speed * (Math.sin(direction) + Math.cos(direction)));
        fl.setPower(speed * (Math.sin(direction) - Math.cos(direction)));
        br.setPower(speed * (Math.sin(direction) - Math.cos(direction)));
        bl.setPower(speed * (Math.sin(direction) + Math.cos(direction)));

    }
}
