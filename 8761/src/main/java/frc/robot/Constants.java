// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double TrackWidth_Meters = 1.0;
    public static final double WheelBase = 1.0;

    public static final int DriveTrain_pidgeon_ID = 0;

    // constants for FL module 
    public static final int Front_Left_Drive = 0;
    public static final int Front_Left_Steer = 0;
    public static final int Front_Left_Steer_Encoder = 0;
    public static final double Front_Left_Soffset = -Math.toRadians(0.0); 

 
    // constants for FR module 
    public static final int Front_Right_Drive = 0;
    public static final int Front_Right_Steer = 0;
    public static final int Front_Right_Steer_Encoder = 0;
    public static final double Front_Right_Soffset = -Math.toRadians(0.0);
    
    // constants for BL module 
    public static final int Back_Left_Drive = 0;
    public static final int Back_Left_Steer = 0;
    public static final int Back_left_Steer_Encoder = 0;
    public static final double Back_Left_Soffset = -Math.toRadians(0.0);
    
    // constants for BR module 
    public static final int Back_Right_Drive = 0;
    public static final int Back_Right_Steer = 0;
    public static final int Back_Right_Steer_Encoder = 0;
    public static final double Back_Right_Soffset = -Math.toRadians(0.0);

}
