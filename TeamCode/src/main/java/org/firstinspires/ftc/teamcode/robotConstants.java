package org.firstinspires.ftc.teamcode;

public class robotConstants {




    public static int maxDifference = 200;
    public static double kP = 0.01;
    public static double kI = 0;
    public static double kD = 0;
    public static double kG = 0.025;
    public static float yawMax = 5;
    public static double Ticks_Per_Rev  = 28;
    public static double GR = 60;
    public static double TPR_CM = 2.2282;
    public static int RPM = 6000;
    public static int maxVel = (int)(robotConstants.RPM * robotConstants.Ticks_Per_Rev / robotConstants.GR);
    public static int maxAccel = (int)(5000);
    public static int maxJerk = (int)(5000);
    public static int max_Ticks_Raw = 200;
    public static double Reduction = .95;
    public static double WHEEL_RADIUS = 1.88976; // in
    public static double TRACK_WIDTH = 16.34; // in

}
