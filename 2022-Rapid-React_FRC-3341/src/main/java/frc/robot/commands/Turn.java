// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import com.kauailabs.navx.frc.AHRS;
public class Turn extends CommandBase {

  private AHRS navx = new AHRS(SPI.Port.kMXP);

  private DriveTrain d;
  private double angle;
  private double error;
  private double speed;
  private int constant = 1;

  /** Creates a new EncoderDrive. */
  public Turn(DriveTrain d, double angle, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.d = d;
    this.angle = angle;
    this.speed = speed;

    if(angle < 0) {
      constant = -1;
    }

    addRequirements(d);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    navx.reset();
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = angle - d.getAngle();
    error = error / angle;
    speed = error * 0.7;

    if(speed > 0.7) {
      speed = 0.7;
    }

    if (speed < 0.1) {
      speed = 0.1;
    }

    d.tankDrive(constant * speed, -constant * speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    d.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return d.getAngle() > angle;
  }

}