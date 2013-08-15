#!/usr/bin/env python

'''
Program:  pretty_print_json.py

This program is used to print the contents of a JSON file
using line breaks and indentation to increase readability.

Usage: pretty_print_json.py <filename>
'''

import sys
import json

def main():

    try:
        jsonFile = sys.argv[1]
    except IndexError:
        print 'Usage: %s <filename>' % sys.argv[0]
        sys.exit(1)

    jsonStr = json.load(open(jsonFile, 'r'))
    print json.dumps(jsonStr, indent=2)


if __name__ == '__main__':
    main()
