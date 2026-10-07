#!/usr/bin/env python3
"""Re-creates the plots of a weld run: plot_weld_log.py /root/shared/logs/weld_<time>"""
import sys

from weld_cell import report

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(1)
    report.make_plots(sys.argv[1])
    print('plots written to', sys.argv[1])
