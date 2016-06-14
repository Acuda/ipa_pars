#!/usr/bin/python
from yaml import load
import cv2
from cv2 import FONT_HERSHEY_COMPLEX, FONT_HERSHEY_SIMPLEX
import sys

class TextToPicture:
    def __init__(self):
        f = open("/home/cme/git/catkin_ws/src/ipa_pars/map_analyzer/knowledge/static-knowledge-base.yaml", 'r')
        yamlfile = load(f)
        f.close()
        self.location_data = yamlfile["location-data"]
        path_to_img = "/home/cme/git/catkin_ws/src/ipa_pars/map_analyzer/maps/logimg6.png"
        self.input_img = cv2.imread(path_to_img, cv2.IMREAD_COLOR)
    
    def putTextOnPic(self):
        index = 0
        for squares in self.location_data:
            print "working on squares % s" % squares["name"]
            text = squares["name"]
            roomY = squares["center"]["X"]
            roomX = squares["center"]["Y"]
            gridscale = 20
            newtext = text.split("-")[1]
            cv2.putText(self.input_img, newtext, (roomX-5,roomY), cv2.FONT_HERSHEY_SIMPLEX, 0.2, (255,255,255),1)
            index += 1
        
        print "we put on %d squares a textname" % index
        try:
            # write image as png [0] no compression --> [10] full compression
            cv2.imwrite("/home/cme/git/catkin_ws/src/ipa_pars/map_analyzer/maps/logwithtext3.png", self.input_img, [cv2.IMWRITE_PNG_COMPRESSION, 10])
            print "write loginfo successful"
        except:
            e = sys.exc_info()[0]
            print e
        
if __name__ == '__main__':
    tp = TextToPicture()
    tp.putTextOnPic()