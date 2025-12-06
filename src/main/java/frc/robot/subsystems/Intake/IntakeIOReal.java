// Copyright 2025-2026 FRC 6985
// https://www.enkatech6985.com/
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.CanIds;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Intake.PivotState;
import frc.robot.Constants.Intake.RollerState;
import frc.robot.util.Utils;

public class IntakeIOReal implements IntakeIO {
  public boolean zeroed = false;
  public final RobotContainer rC;

  public boolean isZeroed() {
    return zeroed;
  }

  public void setZeroed(boolean z) {
    zeroed = z;
  }

  private PivotState realPivotState = PivotState.Up;
  private RollerState realRollerState = RollerState.Off;

  private final TalonFX pivotMotor = new TalonFX(Constants.CanIds.IntakePivotMotor);
  private final TalonFX rollerMotor = new TalonFX(Constants.CanIds.IntakeRollerMotor);
  private final TalonFX centeringMotor = new TalonFX(Constants.CanIds.IntakeCenteringMotor);
  private final DigitalInput linebreak = new DigitalInput(Constants.DioIds.IntakeLineBreak);

  public boolean isZeroed = false;

  public IntakeIOReal(RobotContainer rC) {
    this.rC = rC;
    pivotMotor.getConfigurator().apply(Intake.pivotConfig);
    rollerMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    centeringMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
  }

  // ----------------- GETTERS -----------------

  public PivotState getEffectivePivotState() {
    if (realPivotState != PivotState.OperatorControl)
      return realPivotState;
    if (isUnsafeToGoUp())
      return PivotState.Down;
    if (hasCoral())
      return PivotState.Up;
    if (Superstructure.inputs.wantGroundIntake)
      return PivotState.Down;
    return PivotState.Up;
  }

  public RollerState getEffectiveRollerState() {
    if (realRollerState != RollerState.OperatorControl)
      return realRollerState;
    if (Superstructure.inputs.wantGroundIntake)
      return RollerState.In;
    if (hasCoral())
      return RollerState.SlowIn;
    return RollerState.Off;
  }

  public double getAngle() {
    return pivotMotor.getPosition().getValueAsDouble() * 2 * Math.PI;
  }

  public double getVelocity() {
    return pivotMotor.getVelocity().getValueAsDouble() * 2 * Math.PI;
  }

  public boolean hasCoral() {
    return !linebreak.get() || Controls.operatorController.hid.touchpadButton;
  }

  public boolean atSetpoint() {
    return Math.abs(getAngle() - getEffectivePivotState().angle) < Intake.SETPOINT_THRESHOLD;
  }

  public void zero() {
    pivotMotor.setPosition(0.0);
    isZeroed = true;
  }

  public void setZeroingVoltage() {
    pivotMotor.setVoltage(Intake.ZERO_VOLTAGE);
  }

  public void stop() {
    pivotMotor.setVoltage(0.0);
    rollerMotor.setVoltage(0.0);
    centeringMotor.setVoltage(0.0);
  }

  public void setState(PivotState p, RollerState r) {
    realPivotState = p;
    if (hasCoral() && r == RollerState.Off) {
      if (Controls.superstructureInputs.wantedScoringLevel != Superstructure.ScoringLevel.TROUGH)
        realRollerState = RollerState.In;
      else
        realRollerState = RollerState.SlowIn;
    } else if (hasCoral() && r == RollerState.AlgaeModeIdle) {
      realRollerState = RollerState.SlowIn;
    } else {
      realRollerState = r;
    }
  }

  private boolean isUnsafeToGoUp() {
    return Math.abs(MathUtil.angleModulus(rC.subsystems.arm.getPosition())) < Math.PI
        - rC.subsystems.arm.getElevatorToArm().get(rC.subsystems.elevator.getHeight());
  }

  // ----------------- PERIODIC -----------------

  @Override
  public void periodic() {
    if (!isZeroed)
      return;

    pivotMotor.setControl(new MotionMagicVoltage(getEffectivePivotState().angle / (2 * Math.PI)));
    rollerMotor.setVoltage(getEffectiveRollerState().rollingVoltage);
    centeringMotor.setVoltage(getEffectiveRollerState().centeringVoltage);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Intake angle", () -> Math.toDegrees(getAngle()), v -> {
    });
    builder.addDoubleProperty("Intake setpoint", () -> Math.toDegrees(getEffectivePivotState().angle), v -> {
    });
    builder.addBooleanProperty("at setpoint?", this::atSetpoint, v -> {
    });
    builder.addBooleanProperty("Intake have coral", this::hasCoral, v -> {
    });
    // builder.addDoubleProperty("centering voltage", () ->
    // getEffectiveRollerState().getCenteringVoltage(), v -> {});
    // builder.addBooleanProperty("linebreak", linebreak::get, v -> {});
    builder.addStringProperty("Effective intake pivot state", () -> getEffectivePivotState().toString(), v -> {
    });
    builder.addStringProperty("Underlying intake pivot state", () -> realPivotState.toString(), v -> {
    });
    builder.addBooleanProperty("Is Zeroed?", () -> isZeroed, v -> {
    });
    Utils.addClosedLoopProperties("Intake Pivot", pivotMotor, builder);
    builder.addBooleanProperty("unsafe for intake to go up?", this::isUnsafeToGoUp, v -> {
    });
  }
}
