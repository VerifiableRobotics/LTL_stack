"""
logging for this package temporarily. PIPE to ros later?
Modified from LTLMoP
"""

import logging
import ConfigParser
import sys, os
import time

loggerLevel = 'debug'

def setupLogging(loggerLevel=None):
    # Set up loggers for printing error messages
    class ColorLogFormatter(logging.Formatter):
        def __init__(self, *args, **kwds):
            super(ColorLogFormatter, self).__init__(*args, **kwds)
            self.plain_formatter = logging.Formatter("%(asctime)s.%(msecs)3d %(levelname)5s[%(filename)s:%(lineno)s - %(funcName)20s() ] %(message)s", "%H:%M:%S")
            self.debug_formatter = logging.Formatter("%(asctime)s.%(msecs)3d %(levelname)5s[%(filename)s:%(lineno)s - %(funcName)20s() ] %(message)s", "%H:%M:%S")
            self.detailed_formatter = logging.Formatter("%(asctime)s.%(msecs)3d %(levelname)5s[%(filename)s:%(lineno)s - %(funcName)20s() ] %(message)s", "%H:%M:%S")

        def colorize(self, level, string):
            if sys.platform in ['win32', 'cygwin']:
                # Message with color is not yet supported in Windows
                return string

            else:
                colors = {'ERROR': 91, 'WARNING': 93, 'INFO': 97, 'DEBUG': 94, 'Level 1': 90, 'Level 2': 95, 'Level 4': "7;95", 'Level 6': 96, 'Level 8': 92}
                return "\033[{0}m{1}\033[0m".format(colors[level], string)

        def format(self, record):
            if record.levelname == "INFO":
                precolor = self.plain_formatter.format(record)
            elif record.levelname == "DEBUG":
                precolor = self.debug_formatter.format(record)
            else:
                precolor = self.detailed_formatter.format(record)

            return self.colorize(record.levelname, precolor)

    loggers = {"node": logging.getLogger("node_logger")}

    h = logging.StreamHandler()
    f = ColorLogFormatter()
    h.setFormatter(f)
    for logger in loggers.values():
        if not logger.handlers:
            logger.addHandler(h)

    # also save to file
    # read from terminal: tail -f /var/log/syslog -f /var/tmp/contoller_logger.log
    for logger_name, logger in loggers.iteritems():
        h_file = logging.FileHandler('/var/tmp/'+logger_name+'_logger.log', mode='w')
        h_file.setFormatter(f)
        logger.addHandler(h_file)

    for logger in loggers.values():
        if loggerLevel == 'error':
            logger.setLevel(logging.ERROR)
        elif loggerLevel == 'warning':
            logger.setLevel(logging.WARNING)
        elif loggerLevel == 'info':
            logger.setLevel(logging.INFO)
        elif loggerLevel == 'debug':
            logger.setLevel(logging.DEBUG)
        elif loggerLevel == 'notset':
            #logger.setLevel(logging.NOTSET)
            # for some reason logging.NOTSET does not work
            logger.setLevel(int(1))
        else:
            logger.setLevel(int(loggerLevel))

# Choose the timer func with maximum accuracy for given platform
if sys.platform in ['win32', 'cygwin']:
    best_timer = time.clock
else:
    best_timer = time.time

# Set-up logging automatically on import
setupLogging(loggerLevel)
