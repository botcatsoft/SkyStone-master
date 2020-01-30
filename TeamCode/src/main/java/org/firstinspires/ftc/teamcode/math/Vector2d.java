package org.firstinspires.ftc.teamcode.math;

public class Vector2d {
    public double x, y;
    public Vector2d(double xPos, double yPos)  {
        this.x = xPos;
        this.y = yPos;
    }

    public Vector2d(Vector2d v)  {
        this.x = v.x;
        this.y = v.y;
    }

    public static Vector2d add(Vector2d v, Vector2d w) {
        return new Vector2d(v.x + w.x, v.y + w.y);
    }
    public static Vector2d subtract(Vector2d v, Vector2d w) {
        return new Vector2d(v.x - w.x, v.y - w.y);
    }

    public static Vector2d multiply(Vector2d v, double w) {
        return new Vector2d(v.x * w, v.y * w);
    }

    public static Vector2d multiply(double w, Vector2d v) {
        return new Vector2d(v.x * w, v.y * w);
    }

    public static Vector2d normalize(Vector2d v){
        double magnitude = Math.sqrt(Math.pow(v.x, 2) + Math.pow(v.y, 2));
        double inverseMagnitude = 1/magnitude;
        Vector2d normalizedVector = Vector2d.multiply(v, inverseMagnitude);
        return normalizedVector;
    }


    public static Vector2d rotate(Vector2d v, double angle) {
        double x = v.x * Math.cos(angle) - v.y * Math.sin(angle);
        double y = v.x * Math.sin(angle) - v.y * Math.cos(angle);
        return new Vector2d(x, y);
    }

    public void set(double xPos, double yPos){
        this.x = xPos;
        this.y = yPos;
    }
    public void set(Vector2d j){
        this.x = j.x;
        this.y = j.y;
    }

    public double getMagnitude(){
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

}
