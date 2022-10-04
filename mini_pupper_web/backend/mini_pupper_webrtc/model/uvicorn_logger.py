from enum import Enum
import logging


class UvicornLogger(str, Enum):
    ACCESS = "uvicorn.access"
    ERROR = "uvicorn.error"


logger = logging.getLogger(UvicornLogger.ERROR)
