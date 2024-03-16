// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.visionProcessor;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.VisionProcessor;
import frc.robot.utilities.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SwitchPipeline extends InstantCommand {
  private int m_pipeline;

  public SwitchPipeline(int pipeline) {
    m_pipeline = pipeline;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Limelight.setPipelineIndex(VisionConstants.klimelightName, m_pipeline);
  }
}
