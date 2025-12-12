package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Superstructure;

public class Controls {
  private static final Controls INSTANCE = new Controls();

  public static Controls getInstance() {
    return INSTANCE;
  }

  // Enums
  public enum AlignMode {
    None,
    ReefAlign,
    TroughAlign,
    AlgaeAlign,
    BargeAlign
  }

  // Controllers
  public final XboxController driverController;
  public final XboxController operatorController;

  // State
  public Superstructure.ScoringLevel lastScoringLevel = Superstructure.ScoringLevel.TROUGH;

  // Data class for drive inputs
  public static class DriveInputs {
    public final double forward;
    public final double left;
    public final double rotation;
    public final double deadzone;
    public final AlignMode alignMode;

    public DriveInputs(
        double forward, double left, double rotation, double deadzone, AlignMode alignMode) {
      this.forward = forward;
      this.left = left;
      this.rotation = rotation;
      this.deadzone = deadzone;
      this.alignMode = alignMode;
    }

    public boolean isNonZero() {
      return Math.abs(forward) > deadzone
          || Math.abs(left) > deadzone
          || Math.abs(rotation) > deadzone;
    }

    public DriveInputs redFlipped() {
      return new DriveInputs(-forward, -left, rotation, deadzone, alignMode);
    }
  }

  // Empty inputs constant
  public static final DriveInputs EMPTY_INPUTS = new DriveInputs(0.0, 0.0, 0.0, 0.0, AlignMode.None);

  // Constructor
  private Controls() {
    driverController = new XboxController(0);
    operatorController = new XboxController(1);
  }

  // ==================== XBOX CONTROLLER MAPPING ====================
  /*
   * SÜRÜCÜ KONTROLCÜSÜ (Port 0):
   * - Sol Analog Y → İleri/Geri hareket
   * - Sol Analog X → Sağa/Sola hareket
   * - Sağ Analog X → Dönüş
   * - Sağ Bumper (RB) → Skor (yedek)
   *
   * OPERATÖR KONTROLCÜSÜ (Port 1):
   * - Sol Analog Y → İleri/Geri hareket
   * - Sol Analog X → Sağa/Sola hareket
   * - Sağ Analog X → Dönüş
   * - Sol Analog Basma + Sol Analog X > 0.9 → Kol pozisyonu artır
   * - Sol Analog Basma + Sol Analog X < -0.9 → Kol pozisyonu azalt
   * - D-Pad Yukarı (0°) → Seviye 4
   * - D-Pad Sol (270°) → Seviye 3
   * - D-Pad Aşağı (180°) → Seviye 2
   * - D-Pad Sağ (90°) → Oluk (Trough)
   * - Sol Tetik (LT) > 0.5 → Uzatma (wantExtend)
   * - Sağ Tetik (RT) > 0.5 → Yerden alma (wantGroundIntake)
   * - Sol Bumper (LB) → Yerden yosun alma (wantAlgaeGroundIntake)
   * - Sağ Bumper (RB) → Yosun al (wantGetAlgae)
   * - Sağ Analog Basma → Skor (wantScore)
   * - A Tuşu → Kaynak kol alma (wantArmSourceIntake)
   * - B Tuşu → İşlemci skoru (wantScoreProcessor)
   * - X Tuşu → Kaynak alma (wantSourceIntake)
   * - Y Tuşu → Yosun çıkar (wantDescoreAlgae)
   * - Start Tuşu → Üst yapıyı sıfırla (wantResetSuperstructure)
   */
  // ================================================================

  // Driver inputs
  public DriveInputs getDriverInputs() {
    AlignMode alignMode;
    if (wantBargeAutoAlign()) {
      alignMode = AlignMode.BargeAlign;
    } else if (wantCoralAutoAlign()
        && getSuperstructureInputs().wantedScoringLevel != Superstructure.ScoringLevel.TROUGH) {
      alignMode = AlignMode.ReefAlign;
    } else if (wantCoralAutoAlign()
        && getSuperstructureInputs().wantedScoringLevel == Superstructure.ScoringLevel.TROUGH) {
      alignMode = AlignMode.TroughAlign;
    } else if (wantAlgaeAutoAlign()
        && getSuperstructureInputs().wantGetAlgae
        && !Arm.getInstance().hasObject) {
      alignMode = AlignMode.AlgaeAlign;
    } else {
      alignMode = AlignMode.None;
    }

    return new DriveInputs(
        driverController.getLeftY(), // Sol Analog Y → İleri/Geri
        -driverController.getLeftX(), // Sol Analog X → Sağa/Sola
        -driverController.getRightX(), // Sağ Analog X → Dönüş
        0.05,
        alignMode);
  }

  // Operator inputs
  public DriveInputs getOperatorInputs() {
    AlignMode alignMode;
    if (wantBargeAutoAlign()) {
      alignMode = AlignMode.BargeAlign;
    } else if (wantCoralAutoAlign()
        && getSuperstructureInputs().wantedScoringLevel != Superstructure.ScoringLevel.TROUGH) {
      alignMode = AlignMode.ReefAlign;
    } else if (wantCoralAutoAlign()
        && getSuperstructureInputs().wantedScoringLevel == Superstructure.ScoringLevel.TROUGH) {
      alignMode = AlignMode.TroughAlign;
    } else if (wantAlgaeAutoAlign()
        && getSuperstructureInputs().wantGetAlgae
        && !Arm.getInstance().hasObject) {
      alignMode = AlignMode.AlgaeAlign;
    } else {
      alignMode = AlignMode.None;
    }

    return new DriveInputs(
        -operatorController.getLeftY(), // Sol Analog Y → İleri/Geri
        -operatorController.getLeftX(), // Sol Analog X → Sağa/Sola
        -operatorController.getRightX(), // Sağ Analog X → Dönüş
        0.1,
        alignMode);
  }

  // Auto-align conditions
  public boolean wantCoralAutoAlign() {
    return getSuperstructureInputs().wantExtend;
  }

  public boolean wantAlgaeAutoAlign() {
    return getSuperstructureInputs().wantGetAlgae
        && Arm.getInstance().isAtSetpoint()
        && Elevator.getInstance().isAtSetpoint();
  }

  public boolean wantBargeAutoAlign() {
    return getSuperstructureInputs().wantExtend
        && (Superstructure.getInstance().state == Superstructure.State.AlgaeRest
            || Superstructure.getInstance().state == Superstructure.State.PreBarge
            || Superstructure.getInstance().state == Superstructure.State.ScoreBarge);
  }

  // Offset arm inputs
  public boolean wantOffsetArmPositive() {
    return operatorController.getLeftX() > 0.9 && operatorController.getLeftStickButton();
  }

  public boolean wantOffsetArmNegative() {
    return operatorController.getLeftX() < -0.9 && operatorController.getLeftStickButton();
  }

  // Superstructure inputs
  public Superstructure.SuperstructureInputs getSuperstructureInputs() {
    int pov = operatorController.getPOV();
    Superstructure.ScoringLevel level;

    switch (pov) {
      case 0:
        level = Superstructure.ScoringLevel.L4; // D-Pad Yukarı
        break;
      case 270:
        level = Superstructure.ScoringLevel.L3; // D-Pad Sol
        break;
      case 180:
        level = Superstructure.ScoringLevel.L2; // D-Pad Aşağı
        break;
      case 90:
        level = Superstructure.ScoringLevel.TROUGH; // D-Pad Sağ
        break;
      default:
        level = lastScoringLevel;
    }
    lastScoringLevel = level;

    return new Superstructure.SuperstructureInputs(
        operatorController.getLeftTriggerAxis() > 0.5, // Sol Tetik → Uzatma (wantExtend)

        operatorController.getRightTriggerAxis() > 0.5, // Sağ Tetik → Yerden alma (wantGroundIntake)

        operatorController.getAButton(), // A Tuşu → Kaynak kol alma (wantArmSourceIntake)

        operatorController.getXButton(), // X Tuşu → Kaynak alma (wantSourceIntake)

        driverController.getRightBumperButton() || operatorController.getRightStickButton(), // Sürücü Sağ Bumper veya
                                                                                             // Operatör Sağ Analog
                                                                                             // Basma → Skor (wantScore)
        level, // D-Pad ile seçilen seviye (wantedScoringLevel)

        operatorController.getRightBumperButton(), // Sağ Bumper → Yosun al (wantGetAlgae)

        operatorController.getYButton(), // Y Tuşu → Yosun çıkar (wantDescoreAlgae)
        false, // wantVerticalPickup

        operatorController.getStartButton(), // Start Tuşu → Üst yapıyı sıfırla (wantResetSuperstructure)

        operatorController.getBButton(), // B Tuşu → İşlemci skoru (wantScoreProcessor)

        operatorController.getLeftBumperButton(), // Sol Bumper → Yerden yosun alma (wantAlgaeGroundIntake)
        false // wantPopsiclePickup
    );
  }
}