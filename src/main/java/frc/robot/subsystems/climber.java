// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.CANifier.LEDChannel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.config.*;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.climberConstants;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
//import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class climber extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  double targetRotations = 0.0;
  public static SparkMax climbMotor = new SparkMax(Constants.climberConstants.climberMotorID,MotorType.kBrushless);
  SparkMaxConfig climbMotorConfig = new SparkMaxConfig();
  static SparkClosedLoopController climbPID = climbMotor.getClosedLoopController();
  public RelativeEncoder climbMotorEncoder = climbMotor.getEncoder();
  
  double targetRotationsClimb = 0.0;
  public climber() {

    climbMotorConfig
      .inverted(true)
      .idleMode(IdleMode.kBrake);

      climbMotorConfig.encoder
      .positionConversionFactor(1)
      .velocityConversionFactor(1);

      climbMotorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(Constants.climberConstants.climberP, Constants.climberConstants.climberI, Constants.climberConstants.climberD)
      .outputRange(-0.7, 0.7);

      climbMotorConfig.softLimit
      .forwardSoftLimit(climberConstants.maxLevel)
              .forwardSoftLimitEnabled(true)
      .reverseSoftLimit(climberConstants.minLevel)
            .reverseSoftLimitEnabled(true)
      ;

    
    climbMotor.configure(climbMotorConfig, ResetMode.kResetSafeParameters, null);
      


  }


    public void moveClimber(){
      climbPID.setSetpoint(
            targetRotationsClimb,
            SparkBase.ControlType.kPosition, 
            ClosedLoopSlot.kSlot0,
            0, 
            ArbFFUnits.kVoltage);
    }


    public void spinForward(){
      climbMotor.set(0.1);
    }
    public void spinReverse(){
      climbMotor.set(-0.1);
    }
    public void stopClimb(){
      climbMotor.set(0.0);
    }


    public void climbUp(){
      targetRotationsClimb = climberConstants.highLevel;
      moveClimber();
    }

    public void climbDown(){
      targetRotationsClimb = climberConstants.baseLevel;
      moveClimber();
    }

      //SmartDashboard.putString("elavator forward", "false");
        // this.m_elevatorSys = m_elevatorSys;
        
      
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
    SmartDashboard.putNumber("Climb ENC", climbMotorEncoder.getPosition());
    SmartDashboard.putNumber("Climber P", Constants.climberConstants.climberP);
               SmartDashboard.putNumber("Climber I", Constants.climberConstants.climberI); 
          SmartDashboard.putNumber("Climber D", Constants.climberConstants.climberD);
                Constants.climberConstants.climberP = SmartDashboard.getNumber("Climber P", Constants.climberConstants.climberP);
                Constants.climberConstants.climberI = SmartDashboard.getNumber("Climber I", Constants.climberConstants.climberI); 
                Constants.climberConstants.climberD = SmartDashboard.getNumber("Climber D", Constants.climberConstants.climberD);


  }
}