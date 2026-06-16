#!/usr/bin/env python3
#
# Helper script to convert CSV to MAT.
# For use with data exported by IPGControl.
# Makes it more accessible to MATLAB with less parsing overhead.
#

import sys
import csv
import os
import numpy as np

from os import EX_OK as EXIT_SUCCESS
from scipy.io import savemat

def asextension(path: str, extension: str) -> str:
    base, filename = os.path.split(path)
    comp = filename.split('.')
    name = ".".join(comp[:-1])
    if len(comp) < 2:
        # seems to not have a file name extension
        name = filename
    return os.path.join(base, name + '.' + extension)

def metadata(filepath: str) -> dict:
    with open(filepath) as f:
        reader = csv.reader(f)
        # the first three rows should contain metadata
        meta = {}
        offset = 0
        for row in reader:
            if len(row) and row[0].startswith("#"):
                offset += 1
                # store without field prefix "#"
                meta[row[0][1:].lower()] = row[1:]
            else:
                # remaining rows are data rows
                break
    for i in range(len(meta.get('name', []))):
        # replace dots with underscore in variable names
        old: str = meta['name'][i]
        new = old.replace('.', '_')
        meta['name'][i] = new
    return meta

USAGE_MESSAGE = lambda x: \
f"""
Usage: python3 {x} <csvfile ...>\n
Converts a TruckMaker IPGControl data export file to MAT-files for use with MATLAB.
"""

def usage(program: str) -> None:
    print(USAGE_MESSAGE(program), end='', file=sys.stderr)

def main(argc: int, argv: list) -> int:
    if argc < 2:
        usage(argv[0])
        return -1
    for filepath in argv[1:]:
        print(f"Parsing {filepath}", file=sys.stderr)
        meta = metadata(filepath)
        offs = len(meta.keys())
        # get the numbers as a numpy matrix
        M = np.loadtxt(filepath, delimiter=',', skiprows=offs)
        # ----------------------------------------------
        # save in a format compatible with MATLAB (.mat)
        # ----------------------------------------------
        I = {}
        for i in range(len(meta.get('name', []))):
            I[meta['name'][i]] = i
        # figure out the valid time range
        mask = slice(None)
        if 'Time' in I:
            time = M[:, I['Time']]
            mask = slice(0, np.argmax(time)+1)
            # sanity: check that t_{n+1} >= t_{n}
            diff = np.diff(time[mask])
            if np.any(diff < 0):
                where = np.where(diff < 0)[0].item() + 1
                print(f"Warning: here seems to be discontinuities in the time dimension at index {where}.")
        # savemat takes a dict that represents named lists: {'fieldname': [values...]}
        m = {}
        for name in meta.get('name', []):
            m[name] = M[mask, I[name]]
        outfilepath = asextension(filepath, "mat")
        print(f"Saving  {outfilepath}", file=sys.stderr)
        savemat(outfilepath, m)
    return EXIT_SUCCESS

if __name__ == "__main__":
    argc = len(sys.argv)
    argv = sys.argv
    main(argc, argv)
