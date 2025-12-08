package frc.robot.subsystems.Arm;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants.Arm.PivotState;
import frc.robot.Constants.Arm.RollerState;
import frc.robot.Constants.Arm.Side;

/**
 * Robot Kolu Alt Sisteminin Donanım G/Ç (Input/Output) Arayüzü.
 * * Bu arayüz, gerçek donanım (ArmIOReal) ve simülasyon (ArmIOSim) uygulamaları
 * arasında bir soyutlama katmanı sağlar.
 */
public interface ArmIO {

  PivotState getPivotState();

  RollerState getRollerState();

  boolean isZeroed();

  double getArmOffsetRadians();

  boolean getHasObject();

  boolean isArmStuck();

  /**
   * Motor dönüşleri ve ofset dikkate alınarak Arm'ın radyan cinsinden mevcut
   * konumunu döndürür.
   */
  double getPosition();

  boolean getAtSetpoint();

  /**
   * Arm'ın robot çerçevesinin içinde (yani güvenli bölgede) olup olmadığını
   * kontrol eder.
   */
  boolean isInsideFrame();

  Side getSideCloserToReef();

  Side getSideCloserToBarge();

  Side getSideCloserToProcessor();

  boolean atSafeReefDistance();

  boolean atSafePlacementDistance();

  boolean atSafeBargeDistance();

  boolean atSafeProcessorDistance();

  /**
   * Verilen ham açıyı (radyan) alır ve robotun konumu, elevator yüksekliği
   * ve çevresel kısıtlamalara (respektif resif/reef) göre güvenli,
   * hedeflenen motor pozisyonunu (radyan) hesaplar.
   *
   * @param angle       Hedeflenen ham açı (radyan).
   * @param respectReef Resif kısıtlamalarını dikkate alıp almayacağı.
   * @return Güvenli ve kelepçelenmiş hedeflenen pozisyon (radyan).
   */
  double positionFromAngle(double angle, boolean respectReef);

  /**
   * Mevcut hedef durumuna (PivotState) göre önbelleğe alınmış istenen konumu
   * döndürür.
   * * @return İstenen nihai pozisyon (radyan).
   */
  double getDesiredPosition();

  /**
   * Kolun hedeflenen pivot ve roller durumunu ayarlar.
   */
  void setState(PivotState pivot, RollerState rollers);

  /**
   * Pivot motorun NeutralMode'unu ayarlar (Coast/Brake).
   */
  void setCoastEnabled(boolean coast);

  /**
   * Motor atlamaları gibi durumlar için kol pozisyonuna manuel olarak ofset
   * ekler.
   */
  void offsetArm(double r);

  /**
   * Mutlak enkoderden gelen okumayı, motor dönüş kısıtlamalarına göre
   * kelepçelenmiş ve ofsetlenmiş motor dönüşleri cinsinden döndürür.
   */
  double closeClampedPosition();

  /**
   * Göreli motor pozisyonunu mutlak enkoder değerine göre sıfırlar ve
   * {@code isZeroed} durumunu {@code true} yapar.
   */
  void resetRelativeFromAbsolute();

  /**
   * Periyodik olarak çalıştırılır (örneğin her 20ms). Motorları ayarlar,
   * sensörleri günceller ve durum değişkenlerini hesaplar.
   */
  void periodic();

  /**
   * SendableBuilder kullanarak Sendable verilerini oluşturur
   * (SmartDashboard/Shuffleboard için).
   */
  void initSendable(SendableBuilder builder);

  InterpolatingDoubleTreeMap getElevatorToArm();
}