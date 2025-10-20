
# 3. antenna_mechanism_emulator.py
#!/usr/bin/env python3

#Эмулятор поворотно-наклонного механизма антенны
#Принимает команды управления через UART/RS485 и возвращает текущее положение


import time
import struct
import serial
import threading
from dataclasses import dataclass
from typing import Optional
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


@dataclass
class AntennaPosition:
    #Положение антенны#
    azimuth: float = 90.0      # Азимут (градусы)
    elevation: float = 45.0    # Элевация (градусы)
    
    def __post_init__(self):
        self.azimuth = self.normalize_azimuth(self.azimuth)
        self.elevation = self.clamp_elevation(self.elevation)
    
    @staticmethod
    def normalize_azimuth(angle: float) -> float:
        #Нормализация азимута 0-360#
        return angle % 360.0
    
    @staticmethod
    def clamp_elevation(angle: float) -> float:
        #Ограничение элевации 0-90#
        return max(0.0, min(90.0, angle))


class MAVLinkProtocol:
    #Упрощенный протокол MAVLink для управления антенной#
    
    MAGIC_V1 = 0xFE
    
    @staticmethod
    def parse_command(data: bytes) -> Optional[tuple]:
        #Парсинг команды MAVLink#
        if len(data) < 8:
            return None
        
        try:
            magic = data[0]
            if magic != MAVLinkProtocol.MAGIC_V1:
                return None
            
            length = data[1]
            msg_id = data[5]
            
            # MSG_ID 11 - SET_POSITION
            if msg_id == 11 and length >= 8:
                azimuth, elevation = struct.unpack('<ff', data[6:14])
                return ('set_position', azimuth, elevation)
            
        except Exception as e:
            logger.error(f"Parse error: {e}")
        
        return None
    
    @staticmethod
    def create_position_report(position: AntennaPosition) -> bytes:
        #Создание отчета о положении#
        # Упрощенный формат MAVLink
        payload = struct.pack('<ff', position.azimuth, position.elevation)
        length = len(payload)
        msg_id = 12  # POSITION_REPORT
        
        frame = bytes([
            MAVLinkProtocol.MAGIC_V1,
            length,
            0,  # packet sequence
            1,  # system ID
            1,  # component ID
            msg_id
        ]) + payload
        
        # Простая контрольная сумма
        checksum = sum(frame[1:]) & 0xFFFF
        frame = frame + struct.pack('<H', checksum)
        
        return frame


class RS485Protocol:
    #Протокол RS485 для управления сервоприводами#
    
    START_BYTE = 0xFF
    CMD_SET_POSITION = 0x01
    CMD_GET_POSITION = 0x02
    CMD_SET_SPEED = 0x03
    
    @staticmethod
    def parse_command(data: bytes) -> Optional[tuple]:
        #Парсинг команды RS485#
        if len(data) < 6:
            return None
        
        try:
            if data[0] != RS485Protocol.START_BYTE:
                return None
            
            cmd = data[1]
            
            if cmd == RS485Protocol.CMD_SET_POSITION:
                azimuth, elevation = struct.unpack('<HH', data[2:6])
                return ('set_position', azimuth / 10.0, elevation / 10.0)
            
            elif cmd == RS485Protocol.CMD_GET_POSITION:
                return ('get_position',)
            
            elif cmd == RS485Protocol.CMD_SET_SPEED:
                speed = struct.unpack('<H', data[2:4])[0]
                return ('set_speed', speed / 10.0)
            
        except Exception as e:
            logger.error(f"Parse error: {e}")
        
        return None
    
    @staticmethod
    def create_position_response(position: AntennaPosition) -> bytes:
        #Создание ответа с положением#
        azimuth_raw = int(position.azimuth * 10)
        elevation_raw = int(position.elevation * 10)
        
        response = bytes([
            RS485Protocol.START_BYTE,
            RS485Protocol.CMD_GET_POSITION,
        ]) + struct.pack('<HH', azimuth_raw, elevation_raw)
        
        checksum = sum(response) & 0xFF
        response = response + bytes([checksum])
        
        return response


class AntennaMechanismEmulator:
    #Эмулятор механизма антенны#
    
    def __init__(self, port: str, baudrate: int = 115200, protocol: str = 'RS485'):
        self.port = port
        self.baudrate = baudrate
        self.protocol = protocol  # 'RS485' или 'MAVLink'
        self.serial: Optional[serial.Serial] = None
        self.running = False
        self.thread: Optional[threading.Thread] = None
        
        self.current_position = AntennaPosition()
        self.target_position = AntennaPosition()
        self.movement_speed = 10.0  # градусов в секунду
        
    def connect(self):
        #Подключение к порту#
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.1
            )
            logger.info(f"Antenna mechanism connected to {self.port} "
                       f"at {self.baudrate} baud ({self.protocol})")
            return True
        except Exception as e:
            logger.error(f"Failed to connect: {e}")
            return False
    
    def update_position(self, dt: float):
        #Обновление текущего положения (плавное движение к цели)#
        max_move = self.movement_speed * dt
        
        # Движение азимута
        az_diff = self.target_position.azimuth - self.current_position.azimuth
        
        # Выбор кратчайшего пути для азимута
        if az_diff > 180:
            az_diff -= 360
        elif az_diff < -180:
            az_diff += 360
        
        if abs(az_diff) > max_move:
            az_diff = max_move if az_diff > 0 else -max_move
        
        self.current_position.azimuth = AntennaPosition.normalize_azimuth(
            self.current_position.azimuth + az_diff
        )
        
        # Движение элевации
        el_diff = self.target_position.elevation - self.current_position.elevation
        
        if abs(el_diff) > max_move:
            el_diff = max_move if el_diff > 0 else -max_move
        
        self.current_position.elevation = AntennaPosition.clamp_elevation(
            self.current_position.elevation + el_diff
        )
    
    def process_command(self, data: bytes):
        #Обработка входящей команды#
        if self.protocol == 'RS485':
            cmd = RS485Protocol.parse_command(data)
        else:
            cmd = MAVLinkProtocol.parse_command(data)
        
        if not cmd:
            return
        
        if cmd[0] == 'set_position':
            self.target_position.azimuth = cmd[1]
            self.target_position.elevation = cmd[2]
            logger.info(f"New target: Az={cmd[1]:.1f}° El={cmd[2]:.1f}°")
            
        elif cmd[0] == 'get_position':
            self.send_position()
            
        elif cmd[0] == 'set_speed':
            self.movement_speed = cmd[1]
            logger.info(f"Speed set to {cmd[1]:.1f}°/s")
    
    def send_position(self):
        #Отправка текущего положения#
        if not self.serial or not self.serial.is_open:
            return
        
        try:
            if self.protocol == 'RS485':
                response = RS485Protocol.create_position_response(self.current_position)
            else:
                response = MAVLinkProtocol.create_position_report(self.current_position)
            
            self.serial.write(response)
            logger.debug(f"Sent position: Az={self.current_position.azimuth:.1f}° "
                        f"El={self.current_position.elevation:.1f}°")
            
        except Exception as e:
            logger.error(f"Error sending position: {e}")
    
    def run(self):
        #Основной цикл эмулятора#
        logger.info("Starting antenna mechanism emulator...")
        self.running = True
        
        update_rate = 50  # Гц
        dt = 1.0 / update_rate
        
        while self.running:
            start_time = time.time()
            
            # Чтение команд
            if self.serial and self.serial.in_waiting > 0:
                try:
                    data = self.serial.read(self.serial.in_waiting)
                    self.process_command(data)
                except Exception as e:
                    logger.error(f"Error reading command: {e}")
            
            # Обновление положения
            self.update_position(dt)
            
            # Периодическая отправка положения
            if int(time.time() * 10) % 10 == 0:  # Раз в секунду
                self.send_position()
            
            # Поддержание частоты обновления
            elapsed = time.time() - start_time
            sleep_time = max(0, dt - elapsed)
            time.sleep(sleep_time)
    
    def start(self):
        #Запуск эмулятора в отдельном потоке#
        if self.connect():
            self.thread = threading.Thread(target=self.run, daemon=True)
            self.thread.start()
            return True
        return False
    
    def stop(self):
        #Остановка эмулятора#
        logger.info("Stopping antenna mechanism emulator...")
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        if self.serial and self.serial.is_open:
            self.serial.close()


def main():
    #Тестовый запуск эмулятора#
    import config
    
    port = config.ANTENNA_PORT if not config.EMULATION_MODE else 'loop://'
    emulator = AntennaMechanismEmulator(port, config.ANTENNA_BAUDRATE, 'RS485')
    
    try:
        if emulator.start():
            logger.info("Antenna emulator started. Press Ctrl+C to stop.")
            while True:
                time.sleep(1)
                logger.info(f"Antenna: Az={emulator.current_position.azimuth:.1f}° "
                          f"El={emulator.current_position.elevation:.1f}° "
                          f"(Target: Az={emulator.target_position.azimuth:.1f}° "
                          f"El={emulator.target_position.elevation:.1f}°)")
    except KeyboardInterrupt:
        logger.info("Stopping...")
    finally:
        emulator.stop()


if __name__ == '__main__':
    main()