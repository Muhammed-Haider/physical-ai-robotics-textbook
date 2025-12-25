import logging
import os
from logging.handlers import RotatingFileHandler

# Define a default log file path
LOG_FILE_PATH = os.getenv("LOG_FILE_PATH", "app.log")
# Define default log level (can be overridden by env var)
DEFAULT_LOG_LEVEL = os.getenv("DEFAULT_LOG_LEVEL", "INFO").upper()

def setup_logging():
    """
    Sets up comprehensive logging for the application.
    Logs to console and a rotating file.
    """
    # Create logger
    logger = logging.getLogger("rag_chatbot_backend")
    logger.setLevel(DEFAULT_LOG_LEVEL)

    # Prevent adding multiple handlers if setup_logging is called multiple times
    if not logger.handlers:
        # Console Handler
        console_handler = logging.StreamHandler()
        console_handler.setLevel(DEFAULT_LOG_LEVEL)
        console_formatter = logging.Formatter(
            "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
        )
        console_handler.setFormatter(console_formatter)
        logger.addHandler(console_handler)

        # File Handler (Rotating)
        file_handler = RotatingFileHandler(
            LOG_FILE_PATH,
            maxBytes=1024 * 1024 * 5,  # 5 MB
            backupCount=5
        )
        file_handler.setLevel(DEFAULT_LOG_LEVEL)
        file_formatter = logging.Formatter(
            "%(asctime)s - %(name)s - %(levelname)s - %(module)s:%(funcName)s:%(lineno)d - %(message)s"
        )
        file_handler.setFormatter(file_formatter)
        logger.addHandler(file_handler)

    return logger

# Initialize logger for global use
logger = setup_logging()

# Example usage (can be removed later)
if __name__ == "__main__":
    logger.debug("This is a debug message.")
    logger.info("This is an info message.")
    logger.warning("This is a warning message.")
    logger.error("This is an error message.")
    logger.critical("This is a critical message.")
