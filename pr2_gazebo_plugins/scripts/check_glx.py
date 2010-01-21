#!/usr/bin/env python
import os

if __name__ == "__main__":
  try:
    result = os.system('glxinfo')
    print "bad"
  except:
    exit(0)


  file = os.popen('glxinfo')
  text = file.readlines()
  for tokens in text:
    if tokens.find('server glx') != -1:
      print "found glx server: ",tokens.find('server glx')
      exit(1)
  exit(0)

