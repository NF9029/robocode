package frc.robot.lib.mpu6050;

import frc.robot.lib.mpu6050.MPUData;

import edu.wpi.first.wpilibj.I2C;
import java.nio.ByteBuffer;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.util.sendable.Sendable;

import java.lang.Math;


public class MPU6050 {
  private ByteBuffer m_readBuffer = ByteBuffer.allocate(14);
  private byte m_I2CAddress;
  private I2C m_i2c;

  private double ard_map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  // 0x68
  public MPU6050(byte I2CAddress) {
    m_I2CAddress = I2CAddress;
    m_i2c = new I2C(I2C.Port.kMXP, m_I2CAddress);
    m_i2c.write(0x6B, 0);
  }

  public byte getI2CAddress() {
    return m_I2CAddress;
  }

  public MPUData getMPUData() {
    m_i2c.read(0x3B, 14, m_readBuffer);
    return new MPUData(m_readBuffer);
  }

  public double[] getAllAngles() {
    MPUData data = getMPUData();
    double ax = ard_map(data.m_accX, 265, 402, -90, 90);
    double ay = ard_map(data.m_accY, 265, 402, -90, 90);
    double az = ard_map(data.m_accZ, 265, 402, -90, 90);

    ax = Math.toDegrees(Math.atan2(-ay, -az) + Math.PI);
    ay = Math.toDegrees(Math.atan2(-ax, -az) + Math.PI);
    az = Math.toDegrees(Math.atan2(-ay, -ax) + Math.PI);
  
    return new double[] { ax, ay, az };
  }

  public double getAngleX() {
    MPUData data = getMPUData();
    double ay = ard_map(data.m_accY, 265, 402, -90, 90);
    double az = ard_map(data.m_accZ, 265, 402, -90, 90);

    return Math.toDegrees(Math.atan2(-ay, -az) + Math.PI);
  }

  public double getAngleY() {
    MPUData data = getMPUData();
    double ax = ard_map(data.m_accX, 265, 402, -90, 90);
    double az = ard_map(data.m_accZ, 265, 402, -90, 90);

    return Math.toDegrees(Math.atan2(-ax, -az) + Math.PI);
  }

  public double getAngleZ() {
    MPUData data = getMPUData();
    double ax = ard_map(data.m_accX, 265, 402, -90, 90);
    double ay = ard_map(data.m_accY, 265, 402, -90, 90);

    return Math.toDegrees(Math.atan2(-ay, -ax) + Math.PI);
  }

  public void printAngles() {
    System.out.print(getAngleX());
    System.out.print(", \t");
    System.out.print(getAngleY());
    System.out.print(", \t");
    System.out.print(getAngleX());
    System.out.println();
  }

  public void printAllData() {
    MPUData data = getMPUData();
    System.out.print("Acc: (");
    System.out.print(data.m_accX/16384.);
    System.out.print(", ");
    System.out.print(data.m_accY/16384.);
    System.out.print(", ");
    System.out.print(data.m_accZ/16384.);
    System.out.print(");  ");
    
    System.out.print(data.getTemperature());
    System.out.print("  ");

    System.out.print("Gyro: (");
    System.out.print(data.m_gyroX/131.);
    System.out.print(", ");
    System.out.print(data.m_gyroY/131.);
    System.out.print(", ");
    System.out.print(data.m_gyroZ/131.);
    System.out.println("); ");
  }

  public void printAccData() {
    MPUData data = getMPUData();
    System.out.print("Acc X: ");
    System.out.print(data.m_accX);
    System.out.print(" Acc Y: ");
    System.out.print(data.m_accY);
    System.out.print(" Acc Z: ");
    System.out.println(data.m_accZ);
  }

  public void printTemperature() {
    MPUData data = getMPUData();
    System.out.println(data.getTemperature());
  }

  public void printGyroData() {
    MPUData data = getMPUData();
    System.out.print("Gyro X: ");
    System.out.print(data.m_gyroX);
    System.out.print(" Gyro Y: ");
    System.out.print(data.m_gyroY);
    System.out.print(" Gyro Z: ");
    System.out.println(data.m_gyroZ);
  }
}