// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve_Subsystem;


public class Swerve_Pilot extends CommandBase {
  private final Swerve_Subsystem Swerve;
  
  private DoubleSupplier xSupplier, ySupplier, zSupplier;
  
  public Swerve_Pilot  (Swerve_Subsystem Swervesubsystem,
                        DoubleSupplier X,
                        DoubleSupplier Y,
                        DoubleSupplier Z) {
      this.Swerve = Swervesubsystem;
      this.xSupplier = X;
      this.ySupplier = Y;
      this.zSupplier = Z;

                          addRequirements(Swerve);
    }
  
@Override
  public void execute() {
    Swerve.drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(
        xSupplier.getAsDouble(),
        ySupplier.getAsDouble(),
        zSupplier.getAsDouble(),
        Swerve.getGyroscopeRotation()
        )
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Swerve.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
