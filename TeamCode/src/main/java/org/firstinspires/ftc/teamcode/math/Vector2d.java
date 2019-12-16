package org.firstinspires.ftc.teamcode.math;

public class Vector2d {
    public final double x, y;
    public Vector2d(double xPos, double yPos)  {
        this.x = xPos;
        this.y = yPos;
    }

    public static Vector2d add(Vector2d v, Vector2d w) {
        return new Vector2d(v.x + w.x, v.y + w.y);
    }

    public static Vector2d rotate(Vector2d v, double angle) {
        double x = v.x * Math.cos(angle) - v.y * Math.sin(angle);
        double y = v.x * Math.sin(angle) - v.y * Math.cos(angle);
        return new Vector2d(x, y);
    }
}
