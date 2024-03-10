// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Feeder;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Feeder.FeederReverseOnce;

import frc.robot.subsystems.Feeder;

public class FeedNoteForStorage extends Command {
  Feeder m_feeder;
  boolean detected;
  int counter;

  public FeedNoteForStorage(Feeder feeder) {
    m_feeder = feeder;
    addRequirements(m_feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    detected = false;
    counter=0;
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_feeder.feedNotesToStow();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_feeder.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // if (m_feeder.hasNoteHigher()){
    //   if (detected){
    //     m_feeder.stop();
    //     return true;
    //   } else {
    //     detected=true;
    //     m_feeder.runReverse();
    //     return false;
    //   } 
    // } else {
    //   return false;
    // }
      return m_feeder.hasNoteHigher();
    }
  }
