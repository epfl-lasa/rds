#!/bin/bash
echo "Bash version ${BASH_VERSION}..."
for i in 0.8 1.2 1.6
  do
  	for j in 0.8 1.2 1.6
      do
      	for rho in 0.5 1.5
      	 do
      	   echo $i $j ${rho}
      	   build/demo_rds_4 $i $j ${rho}
      	done
     done
 done