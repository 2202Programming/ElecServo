// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElecServo;

public class Move extends Command {
  /** Creates a new Move. */
  final ElecServo servo;
  public Move() {
     servo = RobotContainer.RC().elec;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("MOVING HERE");
    SmartDashboard.putNumber("P Gain", servo.getkP());
    SmartDashboard.putNumber("I Gain", servo.getkI());
    SmartDashboard.putNumber("D Gain", servo.getkD());
    SmartDashboard.putNumber("I Zone", servo.getkIz());
    SmartDashboard.putNumber("Feed Forward", servo.getkFF());
    // SmartDashboard.putNumber("Max Output", servo.);
    // SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    servo.setkP(SmartDashboard.getNumber("P Gain",0));
    servo.setkI(SmartDashboard.getNumber("I Gain",0));
    servo.setkD(SmartDashboard.getNumber("D Gain",0));
    servo.setkIz(SmartDashboard.getNumber("I zone",0));
    servo.setkFF(SmartDashboard.getNumber("Feed Forward",0));
    servo.setRef(SmartDashboard.getNumber("Set Rotations",0));
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
