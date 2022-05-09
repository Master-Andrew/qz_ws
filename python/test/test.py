import csv

a = "  calibration_time: \"2021-11-06 04:44:21\""


def split_line(line):
    first = -1
    mid = -1
    for c, i in enumerate(line):
        if c != " " and first == -1:
            first = i
        elif c == ":" and mid == -1:
            mid = i
    if first != -1 and mid != -1
        return [line[first:mid],line[mid+2:] ]

