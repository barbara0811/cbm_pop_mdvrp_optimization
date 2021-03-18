#!/usr/bin/env python
# -*- coding: utf-8 -*-

import logging
import rospy

logging.DEBUGS = 8
logging.addLevelName(logging.DEBUGS, "DEBUGS")
logging.__all__ += ['DEBUGS']

logging.DEBUGV = 9
logging.addLevelName(logging.DEBUGV, "DEBUGV")
logging.__all__ += ['DEBUGV']

# These are the sequences need to get colored ouput
RESET_SEQ = "\033[0m"
BOLD_SEQ = "\033[1m"

COLORS = {
    'WARNING': "\033[38;5;130m",
    'INFO': "\033[36m",
    'DEBUG': "\033[38;5;2m",
    'CRITICAL': "\033[31m",
    'ERROR': "\033[31m",
    'DEBUGS': "\033[93m",
    'DEBUGV': "\033[93m"
}

LEVELS = {
    'DEBUG': logging.DEBUG,
    'INFO': logging.INFO,
    'WARN': logging.WARNING,
    'DEBUGS': logging.DEBUGS,
    'DEBUGV': logging.DEBUGV
}


class ColoredFormatter(logging.Formatter):
    """ Output text formatter.
    """
    def __init__(self, msg):
        logging.Formatter.__init__(self, msg)

    def format(self, record):
        skip_line = False
        if record.msg[0] == '\n':
            skip_line = True
            record.msg = record.msg[1:]
        if record.msg[0] == '!':
            bold = BOLD_SEQ
            record.msg = record.msg[1:]
        else:
            bold = ''
        result = logging.Formatter.format(self, record)
        result = bold + COLORS[record.levelname] + result + RESET_SEQ
        if skip_line:
            result = '\n' + result
        return result


class CustomLogger(logging.Logger):
    """
    Customized logger, provides several levels of logging: info, debug, warn,
    critical, error.

    | Usage:
    | my_logger = CustomLogger()
    | my_logger.warn("This is a warning")
    """

    def __init__(self, name='default', level='DEBUG'):
        super(CustomLogger, self).__init__(name, LEVELS[level])

        # create console handler and set level to debug
        ch = logging.StreamHandler()
        ch.setLevel(level)

        # create formatter
        formatter = ColoredFormatter('[%(levelname)s] [%(name)s]> %(message)s')

        # add formatter to ch
        ch.setFormatter(formatter)

        # add ch to logger
        self.addHandler(ch)

    def debugs(self, msg, *args, **kws):
        if self.isEnabledFor(logging.DEBUGS):
            # Yes, logger takes its '*args' as 'args'.
            self._log(logging.DEBUGS, msg, args, **kws)

    def debugv(self, msg, *args, **kws):
        if self.isEnabledFor(logging.DEBUGV):
            # Yes, logger takes its '*args' as 'args'.
            self._log(logging.DEBUGV, msg, args, **kws)


if __name__ == '__main__':
    rospy.init_node('test')

    logger = CustomLogger('test')

    logger.name = 'test_logger'

    logger.debug('debug message')

    logger.info('\ninfo message')
    rospy.loginfo('info')

    logger.warn('warn message')
    rospy.logwarn('warn')

    logger.critical('critical message')
    rospy.logfatal('fatal')

    logger.error('\nerror message')
    rospy.logerr('error')

    logger.debugs('some message')
