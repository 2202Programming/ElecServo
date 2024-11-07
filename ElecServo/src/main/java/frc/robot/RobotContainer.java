// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Move;
import frc.robot.subsystems.ElecServo;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;

import javax.sound.sampled.SourceDataLine;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {
  public final HID_Xbox_Subsystem dc;
  public final ElecServo elec;
  static RobotContainer rc;
public RobotContainer(){
  RobotContainer.rc = this;
    elec = new ElecServo();
    dc = new HID_Xbox_Subsystem(0.3, 0.9, 0.05);
    configureBindings();
}

public static RobotContainer RC(){
  System.out.println("here");
  return rc;
}

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
private void configureBindings() {
  System.out.println("GOT HERE");
    var operator = dc.Operator();
    operator.a().whileTrue(new Move());
  }
}
