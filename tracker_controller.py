
# 4. tracker_controller.py - главный контроллер
#!/usr/bin/env python3
#Главный контроллер антенного трекера
#Обрабатывает телеметрию, принимает решения и управляет антенной

import time
import math
import serial
import struct
import threading
from collections import deque
from dataclasses import dataclass
from typing import Optional, Tuple
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


@dataclass
class TelemetryData:
    #Данные телеметрии#
    rssi1: int = -120
    rssi2: int = -120
    link_quality: int = 0
    timestamp: float = 0.0


@dataclass
class AntennaCommand:
    #Команда для антенны#
    azimuth: float = 0.0
    elevation: float = 0.0


class RSSIFilter:
    #Фильтр RSSI (скользящее среднее)#
    
    def __init__(self, window_size: int = 5):
        self.window_size = window_size
        self.rssi1_buffer = deque(maxlen=window_size)
        self.rssi2_buffer = deque(maxlen=window_size)
    
    def add_sample(self, rssi1: int, rssi2: int):
        #Добавить образец#
        self.rssi1_buffer.append(rssi1)
        self.rssi2_buffer.append(rssi2)
    
    def get_filtered(self) -> Tuple[float, float]:
        #Получить отфильтрованные значения#
        if not self.rssi1_buffer:
            return -120.0, -120.0
        
        avg_rssi1 = sum(self.rssi1_buffer) / len(self.rssi1_buffer)
        avg_rssi2 = sum(self.rssi2_buffer) / len(self.rssi2_buffer)
        
        return avg_rssi1, avg_rssi2


class TrackingAlgorithm:
    #Алгоритм наведения антенны#
    
    def __init__(self, scan_step: float = 5.0, target_accuracy: float = 10.0):
        self.scan_step = scan_step
        self.target_accuracy = target_accuracy
        
        self.mode = 'search'  # 'search', 'track', 'lock'
        self.scan_azimuth = 0.0
        self.scan_elevation = 45.0
        self.scan_direction = 1  # 1 или -1
        
        self.best_rssi = -120
        self.best_azimuth = 0.0
        self.best_elevation = 45.0
        
        # История для определения градиента
        self.rssi_history = deque(maxlen=3)
    
    def update(self, rssi: float, current_az: float, current_el: float) -> AntennaCommand:
        '''
        Обновление алгоритма слежения
        
        Args:
            rssi: Текущий уровень RSSI (дБм)
            current_az: Текущий азимут антенны
            current_el: Текущая элевация антенны
            
        Returns:
            Команда для антенны
        '''
        self.rssi_history.append(rssi)
        
        # Обновление лучшего RSSI
        if rssi > self.best_rssi:
            self.best_rssi = rssi
            self.best_azimuth = current_az
            self.best_elevation = current_el
        
        command = AntennaCommand()
        
        if self.mode == 'search':
            # Режим поиска - сканирование
            command = self._search_mode(rssi, current_az, current_el)
            
            # Переход в режим слежения если сигнал найден
            if rssi > -80:  # Порог обнаружения
                self.mode = 'track'
                logger.info(f"Signal acquired! RSSI={rssi:.0f}dBm, switching to TRACK mode")
        
        elif self.mode == 'track':
            # Режим слежения - движение к максимуму
            command = self._track_mode(rssi, current_az, current_el)
            
            # Переход в режим удержания если точность достаточна
            if abs(current_az - self.best_azimuth) < self.target_accuracy and \
               abs(current_el - self.best_elevation) < self.target_accuracy and \
               rssi > -70:
                self.mode = 'lock'
                logger.info("Target locked!")
            
            # Возврат в поиск если сигнал потерян
            if rssi < -100:
                self.mode = 'search'
                logger.warning("Signal lost! Returning to SEARCH mode")
        
        elif self.mode == 'lock':
            # Режим удержания - точная корректировка
            command = self._lock_mode(rssi, current_az, current_el)
            
            # Возврат в слежение если отклонение большое
            if abs(current_az - self.best_azimuth) > self.target_accuracy * 2:
                self.mode = 'track'
                logger.info("Target deviation detected, switching to TRACK mode")
        
        return command
    
    def _search_mode(self, rssi: float, current_az: float, current_el: float) -> AntennaCommand:
        #Режим поиска сигнала (сканирование)#
        cmd = AntennaCommand()
        
        # Горизонтальное сканирование
        self.scan_azimuth += self.scan_step * self.scan_direction
        
        # Разворот на границах
        if self.scan_azimuth >= 360:
            self.scan_azimuth = 0
        elif self.scan_azimuth < 0:
            self.scan_azimuth = 360
        
        cmd.azimuth = self.scan_azimuth
        cmd.elevation = self.scan_elevation
        
        return cmd
    
    def _track_mode(self, rssi: float, current_az: float, current_el: float) -> AntennaCommand:
        #Режим слежения за сигналом#
        cmd = AntennaCommand()
        
        # Определение градиента RSSI
        if len(self.rssi_history) >= 2:
            rssi_gradient = self.rssi_history[-1] - self.rssi_history[-2]
        else:
            rssi_gradient = 0
        
        # Движение к лучшей известной позиции
        az_error = self.best_azimuth - current_az
        el_error = self.best_elevation - current_el
        
        # Нормализация азимута
        if az_error > 180:
            az_error -= 360
        elif az_error < -180:
            az_error += 360
        
        # Корректировка с учетом градиента
        gain = 0.5 if rssi_gradient > 0 else 0.3
        
        cmd.azimuth = current_az + az_error * gain
        cmd.elevation = current_el + el_error * gain
        
        # Ограничения
        cmd.azimuth = cmd.azimuth % 360
        cmd.elevation = max(0, min(90, cmd.elevation))
        
        return cmd
    
    def _lock_mode(self, rssi: float, current_az: float, current_el: float) -> AntennaCommand:
        #Режим удержания цели#
        cmd = AntennaCommand()
        
        # Малые корректировки для компенсации движения
        # Используем производную RSSI для определения направления
        if len(self.rssi_history) >= 3:
            d_rssi = self.rssi_history[-1] - self.rssi_history[-2]
            
            # Если сигнал ослабевает, делаем микроподстройку
            if d_rssi < -2:
                # Поиск по спирали с малым шагом
                cmd.azimuth = current_az + math.sin(time.time()) * 2
                cmd.elevation = current_el + math.cos(time.time()) * 1
            else:
                # Удержание позиции
                cmd.azimuth = current_az
                cmd.elevation = current_el
        else:
            cmd.azimuth = current_az
            cmd.elevation = current_el
        
        return cmd


class TrackerController:
    #Главный контроллер трекера#
    
    def __init__(self, telemetry_port: str, antenna_port: str,
                 telemetry_baudrate: int = 420000,
                 antenna_baudrate: int = 115200):
        
        self.telemetry_port = telemetry_port
        self.antenna_port = antenna_port
        self.telemetry_baudrate = telemetry_baudrate
        self.antenna_baudrate = antenna_baudrate
        
        self.telemetry_serial: Optional[serial.Serial] = None
        self.antenna_serial: Optional[serial.Serial] = None
        
        self.running = False
        self.thread: Optional[threading.Thread] = None
        
        # Компоненты системы
        self.rssi_filter = RSSIFilter(window_size=5)
        self.tracking_algo = TrackingAlgorithm(scan_step=5.0, target_accuracy=10.0)
        
        # Текущее состояние
        self.current_telemetry = TelemetryData()
        self.current_antenna_position = AntennaCommand(azimuth=90.0, elevation=45.0)
        
        # Буфер для телеметрии
        self.telemetry_buffer = bytearray()
        
    def connect(self) -> bool:
        #Подключение к портам#
        try:
            # Подключение к телеметрии
            self.telemetry_serial = serial.Serial(
                port=self.telemetry_port,
                baudrate=self.telemetry_baudrate,
                timeout=0.1
            )
            logger.info(f"Connected to telemetry: {self.telemetry_port}")
            
            # Подключение к антенне
            self.antenna_serial = serial.Serial(
                port=self.antenna_port,
                baudrate=self.antenna_baudrate,
                timeout=0.1
            )
            logger.info(f"Connected to antenna: {self.antenna_port}")
            
            return True
            
        except Exception as e:
            logger.error(f"Connection failed: {e}")
            return False
    
    def parse_crsf_frame(self, data: bytes) -> Optional[TelemetryData]:
        #Парсинг CRSF фрейма#
        if len(data) < 4:
            return None
        
        try:
            sync = data[0]
            if sync != 0xC8:
                return None
            
            length = data[1]
            frame_type = data[2]
            
            # FRAME_LINK_STATISTICS = 0x14
            if frame_type == 0x14 and length >= 10:
                payload = data[3:3+length-2]
                
                rssi1_raw = payload[0]
                rssi2_raw = payload[1]
                lq = payload[2]
                
                # Конвертация RSSI (offset -130)
                rssi1 = rssi1_raw - 130
                rssi2 = rssi2_raw - 130
                
                telemetry = TelemetryData(
                    rssi1=rssi1,
                    rssi2=rssi2,
                    link_quality=lq,
                    timestamp=time.time()
                )
                
                return telemetry
                
        except Exception as e:
            logger.error(f"CRSF parse error: {e}")
        
        return None
    
    def send_antenna_command(self, command: AntennaCommand):
        #Отправка команды антенне (протокол RS485)#
        if not self.antenna_serial or not self.antenna_serial.is_open:
            return
        
        try:
            # Формат команды RS485
            START_BYTE = 0xFF
            CMD_SET_POSITION = 0x01
            
            azimuth_raw = int(command.azimuth * 10)
            elevation_raw = int(command.elevation * 10)
            
            cmd_data = bytes([START_BYTE, CMD_SET_POSITION]) + \
                      struct.pack('<HH', azimuth_raw, elevation_raw)
            
            checksum = sum(cmd_data) & 0xFF
            cmd_data = cmd_data + bytes([checksum])
            
            self.antenna_serial.write(cmd_data)
            
            logger.debug(f"Sent command: Az={command.azimuth:.1f}° "
                        f"El={command.elevation:.1f}°")
            
        except Exception as e:
            logger.error(f"Error sending command: {e}")
    
    def run(self):
        #Основной цикл контроллера#
        logger.info("Starting tracker controller...")
        self.running = True
        
        update_rate = 50  # Гц
        dt = 1.0 / update_rate
        
        while self.running:
            start_time = time.time()
            
            # Чтение телеметрии
            if self.telemetry_serial and self.telemetry_serial.in_waiting > 0:
                try:
                    chunk = self.telemetry_serial.read(self.telemetry_serial.in_waiting)
                    self.telemetry_buffer.extend(chunk)
                    
                    # Поиск CRSF фреймов
                    while len(self.telemetry_buffer) >= 4:
                        # Поиск sync byte
                        sync_idx = self.telemetry_buffer.find(0xC8)
                        if sync_idx == -1:
                            self.telemetry_buffer.clear()
                            break
                        
                        # Удаление данных до sync
                        if sync_idx > 0:
                            self.telemetry_buffer = self.telemetry_buffer[sync_idx:]
                        
                        # Проверка наличия полного фрейма
                        if len(self.telemetry_buffer) < 4:
                            break
                        
                        frame_len = self.telemetry_buffer[1] + 2
                        if len(self.telemetry_buffer) < frame_len:
                            break
                        
                        # Извлечение фрейма
                        frame = bytes(self.telemetry_buffer[:frame_len])
                        self.telemetry_buffer = self.telemetry_buffer[frame_len:]
                        
                        # Парсинг
                        telemetry = self.parse_crsf_frame(frame)
                        if telemetry:
                            self.current_telemetry = telemetry
                            
                            # Добавление в фильтр
                            self.rssi_filter.add_sample(
                                telemetry.rssi1,
                                telemetry.rssi2
                            )
                            
                except Exception as e:
                    logger.error(f"Error reading telemetry: {e}")
            
            # Получение отфильтрованного RSSI
            avg_rssi1, avg_rssi2 = self.rssi_filter.get_filtered()
            best_rssi = max(avg_rssi1, avg_rssi2)
            
            # Обновление алгоритма слежения
            antenna_cmd = self.tracking_algo.update(
                best_rssi,
                self.current_antenna_position.azimuth,
                self.current_antenna_position.elevation
            )
            
            # Отправка команды антенне
            self.send_antenna_command(antenna_cmd)
            self.current_antenna_position = antenna_cmd
            
            # Логирование
            if int(time.time() * 2) % 2 == 0:  # Раз в 0.5 сек
                logger.info(
                    f"Mode: {self.tracking_algo.mode.upper()} | "
                    f"RSSI: {best_rssi:.0f}dBm | "
                    f"Antenna: Az={antenna_cmd.azimuth:.1f}° "
                    f"El={antenna_cmd.elevation:.1f}°"
                )
            
            # Поддержание частоты обновления
            elapsed = time.time() - start_time
            sleep_time = max(0, dt - elapsed)
            time.sleep(sleep_time)
    
    def start(self):
        #Запуск контроллера#
        if self.connect():
            self.thread = threading.Thread(target=self.run, daemon=True)
            self.thread.start()
            return True
        return False
    
    def stop(self):
        #Остановка контроллера#
        logger.info("Stopping tracker controller...")
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        
        if self.telemetry_serial and self.telemetry_serial.is_open:
            self.telemetry_serial.close()
        if self.antenna_serial and self.antenna_serial.is_open:
            self.antenna_serial.close()


def main():
    #Запуск контроллера#
    import config
    
    telemetry_port = config.TELEMETRY_PORT if not config.EMULATION_MODE \
                    else 'socket://localhost:5000'
    antenna_port = config.ANTENNA_PORT if not config.EMULATION_MODE \
                  else 'socket://localhost:5001'
    
    controller = TrackerController(
        telemetry_port,
        antenna_port,
        config.TELEMETRY_BAUDRATE,
        config.ANTENNA_BAUDRATE
    )
    
    try:
        if controller.start():
            logger.info("Controller started. Press Ctrl+C to stop.")
            while True:
                time.sleep(1)
    except KeyboardInterrupt:
        logger.info("Stopping...")
    finally:
        controller.stop()


if __name__ == '__main__':
    main()