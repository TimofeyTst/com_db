import serial
import sqlite3
import threading
import time
import enum
import logging
from logging.handlers import RotatingFileHandler
from datetime import datetime
from pathlib import Path


class LoggerLevel(str, enum.Enum):
    WARNING = "WARNING"
    INFO = "INFO"
    ERROR = "ERROR"
    DEBUG = "DEBUG"
    CRITICAL = "CRITICAL"
    NOTSET = "NOTSET"


_log_format = (
    "%(asctime)s - [%(levelname)s] - %(name)s - (%(filename)s).%("
    "funcName)s(%(lineno)d) - %(message)s "
)

def get_file_handler(file_name: str) -> RotatingFileHandler:
    Path(file_name).absolute().parent.mkdir(exist_ok=True, parents=True)
    file_handler = RotatingFileHandler(
        filename=file_name,
        maxBytes=5242880,
        backupCount=10,
    )
    file_handler.setFormatter(logging.Formatter(_log_format))
    file_handler.setLevel(LoggerLevel.WARNING.upper())
    return file_handler

def get_stream_handler():
    stream_handler = logging.StreamHandler()
    stream_handler.setFormatter(logging.Formatter(_log_format))
    return stream_handler

def get_logger(name):
    """
    Examples:
        ::

            >>> from app.pkg.logger import get_logger
            >>> logger = get_logger(__name__)
            >>> logger.info("Hello, World!")
            2021-01-01 00:00:00,000 - [INFO] - app.pkg.logger - (logger.py).get_logger(43) - Hello, World!  # pylint: disable=line-too-long
    """
    logger = logging.getLogger(name)
    date_str = datetime.utcnow().strftime('%Y%m__%d')
    file_path = str(
        Path(f"logs/{date_str}.log").absolute(),
    )
    logger.addHandler(get_file_handler(file_name=file_path))
    logger.addHandler(get_stream_handler())
    logger.setLevel(LoggerLevel.INFO.upper())
    return logger


logger = get_logger(__name__)


class SQLiteConnection:
    def __enter__(self):
        self.connection = sqlite3.connect('db.db')
        return self.connection

    def __exit__(self, exc_type, exc_val, exc_tb):
        if exc_type:
            self.connection.rollback()
        else:
            self.connection.commit()
        self.connection.close()


def initialize_database():
    with SQLiteConnection() as conn:
        cursor = conn.cursor()
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS messages (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                is_under_secure BOOLEAN,
                log_level TEXT,
                message TEXT,
                create_ts TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        ''')


def log_message(log_level: str, message: str, is_under_secure: bool):
    with SQLiteConnection() as conn:
        cursor = conn.cursor()
        cursor.execute('''
            INSERT INTO messages (log_level, message, is_under_secure)
            VALUES (?, ?, ?)
        ''', (
            log_level,
            message,
            is_under_secure,
        ))


class MessagePrefix(str, enum.Enum):
    INFO = '[INFO]:'
    ERROR = '[ERROR]:'
    WARNING = '[WARNING]:'
    INTERRUPTED = '[INTERRUPTED]'
    IS_UNDER_SECURE = '[UNDER SECURE]'
    USART_INITIALIZED = 'USART Initialized'


def read_serial(ser):
    buffer = ""
    while True:
        data = ser.read().decode('latin-1')
        if data:
            buffer += data
            if data == '\n':
                line, buffer = buffer.split('\n', 1)
                try:
                    process_line(line.strip())
                except Exception as exc:
                    logger.error("error: %s", exc)
                    buffer = ""


def process_line(line: str):
    is_under_secure = False
    if line.startswith(MessagePrefix.IS_UNDER_SECURE):
        is_under_secure = True
        line = line.replace(MessagePrefix.IS_UNDER_SECURE, '').strip()

    # Проверяем тип сообщения и выполняем соответствующую логику
    if line.startswith(MessagePrefix.INFO):
        message = line.replace(MessagePrefix.INFO, '').strip()
        handle_info(message, is_under_secure)
    elif line.startswith(MessagePrefix.ERROR):
        message = line.replace(MessagePrefix.ERROR, '').strip()
        handle_error(message, is_under_secure)
    elif line.startswith(MessagePrefix.WARNING):
        message = line.replace(MessagePrefix.WARNING, '').strip()
        handle_warning(message, is_under_secure)
    elif line.startswith(MessagePrefix.INTERRUPTED) or line.startswith(MessagePrefix.USART_INITIALIZED):
        logger.info(line)
    else:
        raise Exception(f"Unknown message received: {line}")


def handle_info(message: str, is_under_secure: bool):
    logger.info(f"{is_under_secure}: {message}")
    #log_message(LoggerLevel.INFO, message, is_under_secure)


def handle_warning(message: str, is_under_secure: bool):
    logger.warning(f"{is_under_secure}: {message}")
    log_message(LoggerLevel.WARNING, message, is_under_secure)



def handle_error(message: str, is_under_secure: bool):
    logger.error(f"{is_under_secure}: {message}")
    log_message(LoggerLevel.ERROR, message, is_under_secure)


def main():
    initialize_database()
    ser = serial.Serial('COM2', 9600, timeout=1)
    print("Server starting...")
    read_serial(ser)


if __name__ == "__main__":
    main()
