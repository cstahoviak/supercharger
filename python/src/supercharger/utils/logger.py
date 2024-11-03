"""

"""
import logging

def get_logger(filename: str) -> logging.Logger:
    """

    Args:
        filename: The filename associated with the Logger instance.

    Returns:
        A Logger instance.
    """
    # Instantiate logger object and set level/format
    logger = logging.getLogger(filename)
    logger.setLevel(logging.INFO)
    formatter = logging.Formatter('%(name)s: %(levelname)s: %(message)s')

    # Log to console via StreamHandler
    stream_hdl = logging.StreamHandler()
    stream_hdl.setFormatter(formatter)

    # Add handlers to logger
    logger.addHandler(stream_hdl)

    return logger
