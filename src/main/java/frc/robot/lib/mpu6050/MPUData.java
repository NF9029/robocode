package frc.robot.lib.mpu6050;

import java.nio.ByteBuffer;

public class MPUData {
  public short m_accX, m_accY, m_accZ, m_gyroX, m_gyroY, m_gyroZ;
  private short m_temperature;

  public MPUData(ByteBuffer MPUReading) {
    m_accX = MPUReading.getShort(0);
    m_accY = MPUReading.getShort(2);
    m_accZ = MPUReading.getShort(4);
    m_temperature = MPUReading.getShort(6);
    m_gyroX = MPUReading.getShort(8);
    m_gyroY = MPUReading.getShort(10);
    m_gyroZ = MPUReading.getShort(12);
  }

  public double getTemperature() {
    // Sıcaklığı hesaplamak için biraz işlem
    return m_temperature/340. + 36.53;
  }

}

