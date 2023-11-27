// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import monologue.Logged;

public class RobotContainer implements Logged {
  CommandXboxController controller = new CommandXboxController(0);
  FlywheelSubsystem flywheelSubsystem = new FlywheelSubsystem();
  ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  ArmSubsystem armSubsystem = new ArmSubsystem();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    flywheelSubsystem.setDefaultCommand(flywheelSubsystem.setTarget(100.0));
    controller.a().whileTrue(flywheelSubsystem.setTarget(2000));

    elevatorSubsystem.setDefaultCommand(elevatorSubsystem.setHeight(0.0));
    controller.a().whileTrue(elevatorSubsystem.setHeight(1.0));
    controller.b().whileTrue(elevatorSubsystem.setHeight(2.0));

    armSubsystem.setDefaultCommand(armSubsystem.setAngle(0.0));
    controller.a().whileTrue(armSubsystem.setAngle(Math.PI / 2));
    controller.b().whileTrue(armSubsystem.setAngle(-Math.PI / 3));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
