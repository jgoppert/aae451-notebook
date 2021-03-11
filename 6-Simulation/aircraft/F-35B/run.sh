#!/bin/bash
pkill JSBSim
pkill fgfs
sleep 10
fgfs --fdm=null --native-fdm=socket,in,60,,5550,udp --aircraft=F-35B-jsbsim --airport=SP01 &
sleep 10
JSBSim scripts.xml --realtime --nice &

