package frc.robot.subsystems.Intake;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants.Intake.PivotState;
import frc.robot.Constants.Intake.RollerState;

/**
 * Robot Alım (Intake) Alt Sisteminin Donanım G/Ç (Input/Output) Arayüzü.
 * Bu arayüz, gerçek donanım (IntakeIOReal) tarafından uygulanan tüm genel
 * metotları soyutlar.
 */
public interface IntakeIO {

  // Getter Metotları
  // --------------------------------------------------------------------------------

  /**
   * Intake'in sıfırlanıp sıfırlanmadığını döndürür.
   */
  boolean isZeroed();

  /**
   * Intake'in mevcut ayarlanmış Pivot Durumunu (veya Operatör Kontrolü durumunda
   * mantıksal olarak belirlenen Durumu) döndürür.
   */
  PivotState getEffectivePivotState();

  /**
   * Intake'in mevcut ayarlanmış Roller Durumunu (veya Operatör Kontrolü durumunda
   * mantıksal olarak belirlenen Durumu) döndürür.
   */
  RollerState getEffectiveRollerState();

  /**
   * Intake'in pivot açısını (radyan) döndürür.
   */
  double getAngle();

  /**
   * Intake'in pivot açısı hızını (radyan/saniye) döndürür.
   */
  double getVelocity();

  /**
   * Linebreak sensörünü kullanarak Coral'ın yakalanıp yakalanmadığını kontrol
   * eder.
   */
  boolean hasCoral();

  /**
   * Pivot açısının hedef ayar noktasına ulaşıp ulaşmadığını kontrol eder.
   */
  boolean atSetpoint();

  // Kontrol ve Setter Metotları
  // --------------------------------------------------------------------------------

  /**
   * Intake sıfırlanma durumunu ayarlar.
   */
  void setZeroed(boolean z);

  /**
   * Motorun göreli pozisyonunu sıfırlar.
   */
  void zero();

  /**
   * Pivot motoru sıfırlama için düşük bir voltajla ayarlar.
   */
  void setZeroingVoltage();

  /**
   * Tüm motorları durdurur (0.0 voltaj ayarlar).
   */
  void stop();

  /**
   * Intake'in istenen pivot ve roller durumunu ayarlar.
   * Bu metot, {@code hasCoral()} durumuna göre RollerState'i mantıksal olarak
   * geçersiz kılabilir.
   */
  void setState(PivotState p, RollerState r);

  // Periyodik ve Sendable Metotlar
  // --------------------------------------------------------------------------------

  /**
   * Alt sistemin periyodik olarak çalıştırılacak mantığını içerir (örneğin her
   * 20ms).
   * Motor kontrollerini günceller.
   */
  void periodic();

  /**
   * SendableBuilder kullanarak Sendable verilerini oluşturur
   * (SmartDashboard/Shuffleboard için).
   */
  void initSendable(SendableBuilder builder);
}