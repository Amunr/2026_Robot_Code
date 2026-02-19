// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.intakeConstants;

public class intake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public static SparkMax intakeMotor = new SparkMax(Constants.intakeConstants.intakeMotorID,MotorType.kBrushless);
    SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
    static SparkClosedLoopController intakePID = intakeMotor.getClosedLoopController();
    static FeedForwardConfig intakeFF = new FeedForwardConfig();
    public RelativeEncoder intakeMotorEncoder = intakeMotor.getEncoder();
    double targetSpeed = 0.0;
    public static SparkMax foldMotor = new SparkMax(Constants.intakeConstants.intakeDeployMotorID,MotorType.kBrushless);
    SparkMaxConfig foldMotorConfig = new SparkMaxConfig();
    static SparkClosedLoopController foldPID = foldMotor.getClosedLoopController();
    public RelativeEncoder foldMotorEncoder = foldMotor.getEncoder();
    double targetPos = 0.0;
  public intake() {
    intakeMotorConfig
      .inverted(false)
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(40);

      intakeMotorConfig.encoder
      .positionConversionFactor(1)
      .velocityConversionFactor(1);

      intakeMotorConfig.closedLoop.feedForward
      //.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      //.pid(0.0000, 0, 0)
      .kV(0.001)
      ;
    
    foldMotor.configure(foldMotorConfig, ResetMode.kResetSafeParameters, null);
     
     foldMotorConfig
      .inverted(true)
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(40);

      foldMotorConfig.encoder
      .positionConversionFactor(1)
      .velocityConversionFactor(1);

      foldMotorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(0.08, 0, 0)
      .maxOutput(0.1)
      ;
    
    foldMotor.configure(foldMotorConfig, ResetMode.kResetSafeParameters, null);
     


  }


  public void movePOS (){
    
     //  elevatorPID.setReference(-32,SparkBase.ControlType.kPosition);
          if(targetPos == 0.0){
            foldOut();
          }else{
            foldIn();
          }
            foldPID.setSetpoint(
              targetPos,
              SparkBase.ControlType.kPosition, 
              ClosedLoopSlot.kSlot0,
              0, 
              ArbFFUnits.kVoltage);

            
    }
    public void spinIntake (){
    
     //  elevatorPID.setReference(-32,SparkBase.ControlType.kPosition);
          /* 
            intakePID.setSetpoint(
              targetSpeed,
              SparkBase.ControlType.kVelocity, 
              ClosedLoopSlot.kSlot0,
              0, 
              ArbFFUnits.kVoltage);*/
        intakeMotor.set(targetSpeed);
            
    }

    public void foldIn (){
      targetPos = 0.0; 
      //SmartDashboard.putString("Elevator Set Level", "level One");
    }

  public void foldOut (){
      targetPos = intakeConstants.intakeOutRotations; 
      //SmartDashboard.putString("Elevator Set Level", "level One");
    }

  public void intakeFuel(){
    targetSpeed = Constants.intakeConstants.setIntakeSpeed;
    spinIntake();
  }

  public void extakeFuel(){
    targetSpeed = -Constants.intakeConstants.setIntakeSpeed;
    spinIntake();
  }
  public void stopIntake(){
    if(targetSpeed == -Constants.intakeConstants.setIntakeSpeed){
      targetSpeed = -Constants.intakeConstants.setIntakeSpeed;
    }else{
      targetSpeed = 0.0;
    }
    spinIntake();
  }
  public void stopExtake(){
    if(targetSpeed == Constants.intakeConstants.setIntakeSpeed){
      targetSpeed = Constants.intakeConstants.setIntakeSpeed;
    }else{
      targetSpeed = 0.0;
    }
    spinIntake();
  }
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("foldMotorVelocity", foldMotorEncoder.getPosition());
    SmartDashboard.putNumber("foldMotorSetpoint", targetPos);
    SmartDashboard.putNumber("intakeMotorVelocity", intakeMotorEncoder.getVelocity());
    SmartDashboard.putNumber("intakeMotorSetpoint", targetSpeed);

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
