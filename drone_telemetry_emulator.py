# 2. drone_telemetry_emulator.py
#!/usr/bin/env python3

#Эмулятор телеметрии с пульта ELRS/CRSF
#Имитирует движение дрона и передачу телеметрии через UART

import time
import math
import struct
import serial
import threading
from dataclasses import dataclass
from typing import Optional
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


@dataclass
class DroneState:
    #Состояние дрона#
    azimuth: float = 0.0        # Азимут от антенны (градусы)
    elevation: float = 45.0     # Элевация от антенны (градусы)
    distance: float = 500.0     # Расстояние до антенны (метры)
    rssi1: int = -50           # RSSI антенна 1 (dBm)
    rssi2: int = -52           # RSSI антенна 2 (dBm)
    link_quality: int = 100     # Качество связи (%)
    battery_voltage: float = 12.6  # Напряжение батареи (В)


class CRSFProtocol:
    #Протокол CRSF для телеметрии#
    
    SYNC_BYTE = 0xC8
    
    # Типы фреймов CRSF
    FRAME_GPS = 0x02
    FRAME_BATTERY = 0x08
    FRAME_LINK_STATISTICS = 0x14
    FRAME_ATTITUDE = 0x1E
    
    @staticmethod
    def calculate_crc(data: bytes) -> int:
        #Вычисление CRC для CRSF#
        crc = 0
        for byte in data:
            crc = crc ^ byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0xD5
                else:
                    crc = crc << 1
                crc = crc & 0xFF
        return crc
    
    @staticmethod
    def create_link_stats_frame(state: DroneState) -> bytes:
        #Создание фрейма статистики связи#
        payload = struct.pack(
            '<BBBBBBBBBb',
            state.rssi1 + 130,      # Uplink RSSI 1 (offset)
            state.rssi2 + 130,      # Uplink RSSI 2
            state.link_quality,     # Uplink Link Quality
            -70 + 130,             # Downlink RSSI
            100,                   # Downlink Link Quality
            0,                     # Uplink SNR
            0,                     # Active antenna
            0,                     # RF Mode
            0,                     # Uplink TX Power
            0                      # Downlink SNR
        )
        
        frame_type = CRSFProtocol.FRAME_LINK_STATISTICS
        length = len(payload) + 2  # payload + type + crc
        
        frame = bytes([CRSFProtocol.SYNC_BYTE, length, frame_type]) + payload
        crc = CRSFProtocol.calculate_crc(frame[2:])
        frame = frame + bytes([crc])
        
        return frame
    
    @staticmethod
    def create_battery_frame(state: DroneState) -> bytes:
        #Создание фрейма данных батареи#
        voltage = int(state.battery_voltage * 10)  # В десятых вольта
        current = int(5.0 * 10)  # 5A в десятых ампера
        capacity = int(2200)     # mAh
        remaining = 75           # %
        
        payload = struct.pack(
            '<HHxBB',
            voltage,
            current,
            capacity & 0xFF,
            (capacity >> 8) | (remaining << 1)
        )
        
        frame_type = CRSFProtocol.FRAME_BATTERY
        length = len(payload) + 2
        
        frame = bytes([CRSFProtocol.SYNC_BYTE, length, frame_type]) + payload
        crc = CRSFProtocol.calculate_crc(frame[2:])
        frame = frame + bytes([crc])
        
        return frame


class DroneTelemetryEmulator:
    #Эмулятор телеметрии дрона#
    
    def __init__(self, port: str, baudrate: int = 420000):
        self.port = port
        self.baudrate = baudrate
        self.serial: Optional[serial.Serial] = None
        self.running = False
        self.state = DroneState()
        self.thread: Optional[threading.Thread] = None
        
        # Параметры траектории
        self.trajectory_time = 0.0
        self.trajectory_type = 'circle'  # 'circle', 'line', 'random'
        
    def connect(self):
        #Подключение к порту#
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1.0
            )
            logger.info(f"Connected to {self.port} at {self.baudrate} baud")
            return True
        except Exception as e:
            logger.error(f"Failed to connect: {e}")
            return False
    
    def update_drone_position(self, dt: float):
        #Обновление позиции дрона по траектории#
        self.trajectory_time += dt
        t = self.trajectory_time
        
        if self.trajectory_type == 'circle':
            # Круговая траектория
            radius = 45  # градусы
            center_azimuth = 180
            center_elevation = 45
            angular_speed = 10  # градусов в секунду
            
            angle = (angular_speed * t) % 360
            angle_rad = math.radians(angle)
            
            self.state.azimuth = center_azimuth + radius * math.cos(angle_rad)
            self.state.elevation = center_elevation + radius * 0.3 * math.sin(angle_rad)
            self.state.distance = 500 + 100 * math.sin(angle_rad)
            
        elif self.trajectory_type == 'line':
            # Линейная траектория
            self.state.azimuth = (90 + 90 * math.sin(t * 0.2)) % 360
            self.state.elevation = 30 + 20 * math.sin(t * 0.3)
            self.state.distance = 400 + 200 * math.sin(t * 0.1)
        
        # Обновление RSSI на основе расстояния и направления
        # Простая модель затухания сигнала
        base_rssi = -30
        distance_loss = 20 * math.log10(max(self.state.distance, 10) / 100)
        self.state.rssi1 = int(base_rssi - distance_loss + math.sin(t) * 5)
        self.state.rssi2 = int(base_rssi - distance_loss + math.cos(t) * 5 - 2)
        
        # Ограничение RSSI
        self.state.rssi1 = max(-120, min(-30, self.state.rssi1))
        self.state.rssi2 = max(-120, min(-30, self.state.rssi2))
        
        # Link quality зависит от RSSI
        lq = 100 + int(self.state.rssi1 + 70)
        self.state.link_quality = max(0, min(100, lq))
    
    def send_telemetry(self):
        #Отправка телеметрии через UART
        if not self.serial or not self.serial.is_open:
            return
        
        try:
            # Отправляем фрейм статистики связи (содержит RSSI)
            link_frame = CRSFProtocol.create_link_stats_frame(self.state)
            self.serial.write(link_frame)
            
            # Отправляем фрейм батареи
            battery_frame = CRSFProtocol.create_battery_frame(self.state)
            self.serial.write(battery_frame)
            
            logger.debug(f"Sent telemetry: Az={self.state.azimuth:.1f}° "
                        f"El={self.state.elevation:.1f}° "
                        f"RSSI1={self.state.rssi1}dBm "
                        f"RSSI2={self.state.rssi2}dBm")
            
        except Exception as e:
            logger.error(f"Error sending telemetry: {e}")
    
    def run(self):
        #Основной цикл эмуляции
        logger.info("Starting telemetry emulator...")
        self.running = True
        
        update_rate = 50  # Гц (ELRS обычно 50Hz)
        dt = 1.0 / update_rate
        
        while self.running:
            start_time = time.time()
            
            # Обновление позиции дрона
            self.update_drone_position(dt)
            
            # Отправка телеметрии
            self.send_telemetry()
            
            # Поддержание частоты обновления
            elapsed = time.time() - start_time
            sleep_time = max(0, dt - elapsed)
            time.sleep(sleep_time)
    
    def start(self):
        #Запуск эмулятора в отдельном потоке
        if self.connect():
            self.thread = threading.Thread(target=self.run, daemon=True)
            self.thread.start()
            return True
        return False
    
    def stop(self):
        #Остановка эмулятора
        logger.info("Stopping telemetry emulator...")
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        if self.serial and self.serial.is_open:
            self.serial.close()
    
    def set_trajectory(self, trajectory_type: str):
        #Установка типа траектории
        if trajectory_type in ['circle', 'line', 'random']:
            self.trajectory_type = trajectory_type
            self.trajectory_time = 0.0
            logger.info(f"Trajectory changed to: {trajectory_type}")


def main():
    #Тестовый запуск эмулятора
    import config
    
    port = config.TELEMETRY_PORT if not config.EMULATION_MODE else 'loop://'
    emulator = DroneTelemetryEmulator(port, config.TELEMETRY_BAUDRATE)
    
    try:
        if emulator.start():
            logger.info("Emulator started. Press Ctrl+C to stop.")
            while True:
                time.sleep(1)
                logger.info(f"Drone: Az={emulator.state.azimuth:.1f}° "
                          f"El={emulator.state.elevation:.1f}° "
                          f"RSSI={emulator.state.rssi1}dBm")
    except KeyboardInterrupt:
        logger.info("Stopping...")
    finally:
        emulator.stop()


if __name__ == '__main__':
    main()