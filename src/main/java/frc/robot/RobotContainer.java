// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.ModuleConstants.DriveConstants;
import frc.robot.ModuleConstants.OIConstants;

public class RobotContainer {
  
  private final swerveSubsystemBase swerveSubsytem = new swerveSubsystemBase();
  private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);


  public RobotContainer() {
    swerveSubsytem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsytem,
    ()-> -driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
    ()-> driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
    ()-> driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
    ()-> driverJoytick.getRawButton( OIConstants.kDriverFieldOrientedButtonIdx )));
    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(driverJoytick, 1).whenPresed(()->swerveSubsytem.zeroHeading);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
