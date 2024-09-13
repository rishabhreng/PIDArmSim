// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Arm;

public class RobotContainer {
  Arm arm = new Arm();
  CommandXboxController controller = new CommandXboxController(0);

  double targetAngle = 0.0;
  double targetRPM = 0.0;

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    controller.povUp().whileTrue((Commands.run(() -> targetAngle += 0.005).alongWith(arm.setTargetPosition(() -> targetAngle))).repeatedly());
    controller.povDown().whileTrue((Commands.run(() -> targetAngle -= 0.005).alongWith(arm.setTargetPosition(() -> targetAngle))).repeatedly());

    controller.povRight().whileTrue((Commands.run(() -> targetRPM += 1).alongWith(arm.setTargetVelocity(() -> targetRPM))).repeatedly());
    controller.povLeft().whileTrue((Commands.run(() -> targetRPM -= 1).alongWith(arm.setTargetVelocity(() -> targetRPM))).repeatedly());

    controller.a().onTrue(arm.sysIDQuasistatic(Direction.kForward));
    controller.b().onTrue(arm.sysIDQuasistatic(Direction.kReverse));
    controller.x().onTrue(arm.sysIDDynamic(Direction.kForward));
    controller.y().onTrue(arm.sysIDDynamic(Direction.kReverse).finallyDo(()-> SignalLogger.stop()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
