#! /usr/bin/env python

"""
logging for this package temporarily. PIPE to ros later?
Modified from LTLMoP
"""

import logging
import ConfigParser
import sys, os
import time

import slugs_logging

loggerLevel = "debug"

def setupLogging(loggerLevel=None):
    # Set up loggers for printing error messages
    class ColorLogFormatter(logging.Formatter):
        def __init__(self, *args, **kwds):
            super(ColorLogFormatter, self).__init__(*args, **kwds)
            self.plain_formatter = logging.Formatter("%(asctime)s.%(msecs)3d %(levelname)5s [ %(module)s ] %(message)s", "%H:%M:%S")
            self.debug_formatter = logging.Formatter("%(asctime)s.%(msecs)3d %(levelname)5s [ %(module)s ] %(message)s (line %(lineno)s)", "%H:%M:%S")
            self.detailed_formatter = logging.Formatter("%(asctime)s.%(msecs)3d %(levelname)5s[ %(module)s ] %(message)s  (%(pathname)s, line %(lineno)s)", "%H:%M:%S")
        def colorize(self, level, string):
            if sys.platform in ['win32', 'cygwin']:
                # Message with color is not yet supported in Windows
                return string

            else:
                colors = {'ERROR': 91, 'WARNING': 93, 'INFO': 97, 'DEBUG': 94, 'Level 1': 100, 'Level 2': 105, 'Level 4': 104, 'Level 6': 102, 'Level 8': 101}
                return "\033[{0}m{1}\033[0m".format(colors[level], string)

        def format(self, record):
            if record.levelname == "INFO":
                precolor = self.plain_formatter.format(record)
            elif record.levelname == "DEBUG":
                precolor = self.debug_formatter.format(record)
            else:
                precolor = self.detailed_formatter.format(record)

            # indent when necessary
            header, footer = precolor.split(record.message)
            precolor = precolor.replace('\n', '\n' + ' '*len(header))

            return self.colorize(record.levelname, precolor)

    slugs_logger = logging.getLogger("slugs_logger")
    h = logging.StreamHandler()
    f = ColorLogFormatter()
    h.setFormatter(f)
    if not slugs_logger.handlers:
        slugs_logger.addHandler(h)

    # also save to file
    # read from terminal: tail -f /var/log/syslog -f /var/tmp/slugs_logger.log
    h_file = logging.FileHandler('/var/tmp/slugs_logger.log', mode='w')
    h_file.setFormatter(f)
    slugs_logger.addHandler(h_file)

    if loggerLevel == 'error':
        slugs_logger.setLevel(logging.ERROR)
    elif loggerLevel == 'warning':
        slugs_logger.setLevel(logging.WARNING)
    elif loggerLevel == 'info':
        slugs_logger.setLevel(logging.INFO)
    elif loggerLevel == 'debug':
        slugs_logger.setLevel(logging.DEBUG)
    elif loggerLevel == 'notset':
        #slugs_logger.setLevel(logging.NOTSET)
        # for some reason logging.NOTSET does not work
        slugs_logger.setLevel(int(1))
    else:
        slugs_logger.setLevel(int(loggerLevel))

# Choose the timer func with maximum accuracy for given platform
if sys.platform in ['win32', 'cygwin']:
    best_timer = time.clock
else:
    best_timer = time.time

# Set-up logging automatically on import
setupLogging(loggerLevel)
