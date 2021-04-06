#!/bin/bash
pkill fgfs
fgfs --fdm=null --fg-aircraft=./aircraft --native-fdm=socket,in,60,,5550,udp --aircraft=F-35B-2 --airport=KLAF --timeofday=noon

