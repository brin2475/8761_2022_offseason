// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;





import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;




import static frc.robot.Constants.*;



public class Swerve_Subsystem extends SubsystemBase {

  public static final double MAX_VOLTAGE = 12.0;

// MVMPS(Max. Velocity. Meters. Per. Second)
  public static final double MVMPS = 6380.0 / 0 *
  SdsModuleConfigurations.MK4_L2.getDriveReduction() *
  SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;
// AVRPS (MAX. ANGULAR. VELOCITY. RADIANS. PER. SECOND)
  public static final double AVRPS = MVMPS /
  Math.hypot( TrackWidth_Meters/ 2.0, WheelBase / 2.0);

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    //Front left 
    new Translation2d(TrackWidth_Meters / 2.0, WheelBase / 2.0),
    //front right
    new Translation2d(TrackWidth_Meters / 2.0, -WheelBase / 2.0),
    //back left 
    new Translation2d(-TrackWidth_Meters / 2.0, WheelBase / 2.0),
    //back right 
    new Translation2d(-TrackWidth_Meters / 2.0, WheelBase / 2.0)
  );

  private final AHRS m_navx = new AHRS(Port.kMXP); 

  private final SwerveModule FLModule;
  private final SwerveModule FRmodule;
  private final SwerveModule BLmodule;
  private final SwerveModule BRmodule;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  
  public Swerve_Subsystem() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    FLModule = Mk4SwerveModuleHelper.createFalcon500Neo(
      tab.getLayout("FLmodule", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(0, 0),
            
            Mk4SwerveModuleHelper.GearRatio.L2,
             Front_Left_Drive, 
             Front_Left_Steer,
             Front_Left_Steer_Encoder,
             Front_Right_Soffset
             
);

    FRmodule = Mk4SwerveModuleHelper.createFalcon500Neo(
      tab.getLayout("FRmodule", BuiltInLayouts.kList)
              .withSize(2, 4)
              .withPosition(2, 0),
               
              Mk4SwerveModuleHelper.GearRatio.L2,
               Front_Right_Drive,
               Front_Right_Steer,
               Front_Right_Steer_Encoder,
               Front_Right_Soffset
               
);

    BLmodule = Mk4SwerveModuleHelper.createFalcon500Neo(
      tab.getLayout("BLmodule", BuiltInLayouts.kList)
                  .withSize(2, 4)
                  .withPosition(4, 0), 
                  Mk4SwerveModuleHelper.GearRatio.L2,
                  Back_Left_Drive, 
                  Back_Left_Steer,
                  Back_left_Steer_Encoder,
                  Back_Left_Soffset
                  
);  

    BRmodule = Mk4SwerveModuleHelper.createFalcon500Neo(
      tab.getLayout("BRmodule", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0),
                    Mk4SwerveModuleHelper.GearRatio.L2,
                    Back_Right_Drive, 
                    Back_Right_Steer,
                    Back_Right_Steer_Encoder,
                    Back_Right_Soffset 
  );
}

  public void zeroGyroscope() {
    m_navx.zeroYaw();
  }

  public Rotation2d getGyroscopeRotation() {
    if (m_navx.isMagnetometerCalibrated()) {
            
            return Rotation2d.fromDegrees(m_navx.getFusedHeading());
          }
      return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
  }


  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }

  

  
   


  
  
  
  
  
  
  
  
  
  @Override
  public void periodic() {
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(m_chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MVMPS);

    FLModule.set(states[0].speedMetersPerSecond / MVMPS * MAX_VOLTAGE, states[0].angle.getRadians());
    FRmodule.set(states[1].speedMetersPerSecond / MVMPS * MAX_VOLTAGE, states[1].angle.getRadians());
    BLmodule.set(states[2].speedMetersPerSecond / MVMPS * MAX_VOLTAGE, states[2].angle.getRadians());
    BRmodule.set(states[3].speedMetersPerSecond / MVMPS * MAX_VOLTAGE, states[2].angle.getRadians());
  
  
  }
}
