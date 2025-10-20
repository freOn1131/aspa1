# 1. config.py - конфигурация системы
#Конфигурационный файл для антенного трекера ELRS

# Настройки портов
TELEMETRY_PORT = '/dev/ttyAMA0'  # Порт для приема телеметрии (UART)
TELEMETRY_BAUDRATE = 420000      # Скорость ELRS (420 kbaud)

ANTENNA_PORT = '/dev/ttyAMA1'    # Порт управления антенной (RS485/UART)
ANTENNA_BAUDRATE = 115200

# Для эмуляции используем виртуальные порты
EMULATION_MODE = True
TELEMETRY_EMULATOR_PORT = 'pty://localhost:5000'
ANTENNA_EMULATOR_PORT = 'pty://localhost:5001'

# Параметры антенны
ANTENNA_MIN_AZIMUTH = 0      # Минимальный угол азимута (градусы)
ANTENNA_MAX_AZIMUTH = 360    # Максимальный угол азимута
ANTENNA_MIN_ELEVATION = 0    # Минимальный угол элевации
ANTENNA_MAX_ELEVATION = 90   # Максимальный угол элевации

# Параметры алгоритма наведения
TARGET_ACCURACY = 10         # Требуемая точность наведения (градусы)
RSSI_THRESHOLD = -90         # Минимальный уровень сигнала (dBm)
SCAN_STEP = 5                # Шаг сканирования при поиске (градусы)
SCAN_SPEED = 2               # Скорость сканирования (градусы/сек)
TRACKING_SPEED = 5           # Скорость слежения (градусы/сек)

# Фильтрация сигнала
RSSI_FILTER_WINDOW = 5       # Размер окна скользящего среднего
PREDICTION_ENABLED = True    # Включить предиктивный фильтр

# Веб-интерфейс
WEB_HOST = '0.0.0.0'
WEB_PORT = 8080
WEB_UPDATE_RATE = 10         # Частота обновления (Гц)

# Логирование
LOG_LEVEL = 'INFO'           # DEBUG, INFO, WARNING, ERROR
LOG_FILE = 'tracker.log'