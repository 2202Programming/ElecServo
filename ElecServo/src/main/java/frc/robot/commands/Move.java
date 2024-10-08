// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElecServo;

public class Move extends Command {
  /** Creates a new Move. */
  public Move() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
      ElecServo servo;
  @Override
  public void initialize() {
     servo = new ElecServo();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    servo.move();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
