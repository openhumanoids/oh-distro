import os
import datetime as dt
import subprocess
import tempfile
import argparse
from functools import total_ordering


@total_ordering
class LCMLog(object):
    def __init__(self, path):
        self.path = path
        self.start = dt.datetime.fromtimestamp(os.path.getmtime(path))

    def __lt__(self, other):
        if isinstance(other, LCMLog):
            return self.start < other.start
        else:
            return self.start < other

    def __eq__(self, other):
        if isinstance(other, LCMLog):
            return self.start == other.start
        else:
            return self.start == other

    def __repr__(self):
        return "LCMLog at location {:s} started at {:s}".format(self.path, self.start.isoformat())


class NoLogsAvailableError(Exception):
    pass

class RollingLogDatabase(object):
    def __init__(self, folder):
        self.folder = folder

    def _loadLogs(self):
        return [LCMLog(os.path.join(self.folder, p)) for p in os.listdir(self.folder) if p.startswith("log_split")]

    def getLogsInRange(self, start, stop=None):
        if stop is None:
            stop = dt.datetime.now()
        return [log for log in sorted(self._loadLogs()) if log >= start and log <= stop]

    def spliceLogs(self, logs, outfile=None):
        if len(logs) == 0:
            raise NoLogsAvailableError("No logs available to splice")
        if outfile is None:
            outfile = tempfile.mkstemp(suffix=".lcm")[1]
        if len(logs) == 1:
            cmd = "cp"
        else:
            cmd = "bot-lcm-logsplice"
        subprocess.check_call([cmd] + [log.path for log in logs] + [outfile])
        return outfile


def main():
    parser = argparse.ArgumentParser(description="Splice together the rolling logs for the last h hours, m minutes, and s seconds")
    parser.add_argument("--seconds", "-S", metavar="S", type=float, nargs=1, default=[0])
    parser.add_argument("--minutes", "-M", metavar="M", type=float, nargs=1, default=[0])
    parser.add_argument("--hours", "-H", metavar="H", type=float, nargs=1, default=[0])
    args = parser.parse_args()
    db = RollingLogDatabase(os.path.abspath(os.curdir))
    start = dt.datetime.now() - dt.timedelta(seconds=args.seconds[0],
                                                              minutes=args.minutes[0],
                                                              hours=args.hours[0])
    stop = dt.datetime.now()
    logs = db.getLogsInRange(start=start, stop=stop)
    print db.spliceLogs(logs, outfile="lcmlog_from_{:s}_to_{:s}".format(start.isoformat(), stop.isoformat()).replace(':', '.'))

if __name__ == '__main__':
    main()

