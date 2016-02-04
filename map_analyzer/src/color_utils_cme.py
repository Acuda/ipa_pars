#!/usr/bin/python

import random


# list of different colors:
# from www.visual-graphics.de/de/service/ral-classic-farbtabelle
#
COLORS = [(205,186,136), # RAL 1000 Gruenbeige
          (208,176,132), # RAL 1001 Beige
          (249,168,0),   # RAL 1003 Signalgelb
          (203,142,30),  # RAL 1005 Honiggelb
          (175,128,79),  # RAL 1011 Braunbeige
          (221,175,39),  # RAL 1012 Zitronengelb
          (221,196,154), # RAL 1014 Elfenbein
          (241,221,56),  # RAL 1016 Schwefelgelb
          (246,169,80),  # RAL 1017 Safrangelb
          (255,255,0),   # RAL 1026 Leuchtgelb
          (167,127,14),  # RAL 1027 Currygelb
          (255,155,0),   # RAL 1028 Melonengelb
          (191,72,27),   # RAL 2002 Blutorange
          (246,120,40),  # RAL 2003 Pastellorange
          (255,77,6),    # RAL 2005 Leuchtorange
          (213,101,14),  # RAL 2012 Lachsorange
          (146,62,37),   # RAL 2013 Perlorange
          (167,41,32),   # RAL 3000 Feuerrot
          (89,25,31),    # RAL 3005 Weinrot
          (203,115,117), # RAL 3014 Altrosa
          (187,30,16),   # RAL 3020 Verkehrsrot
          (255,42,27),   # RAL 3026 Leuchthellrot
          (171,39,60)    # RAL 3027 Himbeerrot
          ]

def shuffle_list(colors):
    random.shuffle(colors)
    return colors

def randomColor(scale):
    colorlist = []
    for i in range(30, 240, scale):
        for j in range(30, 240, scale):
            for k in range(30,240, scale):
                color = []
                color.append(i)
                color.append(j)
                color.append(k)
                colorlist.append(color)
    print str(len(colorlist))+" colors added"
    return colorlist
