#!/usr/bin/env python


string="0xff"
if not string.isdigit():
    print("ERR")
else:
    HEXx=int(string,16)
    print(HEXx)