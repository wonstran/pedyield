ABOUT

This respo is a tool produced from a research project that aims to identify the behaviors of turning vehicles yeilding pedestrians/bicyclists/scooters at inertersections using video processing technologies. Yielding behaviroes are an improtant surregate safety measures to represent pedestrian safety at urban intersections. This tool provides a rapid and low-cost safety diagnosis approach for proactive pedestrian safety analysis.

The tool include four components:

1. object detection (YOLO V3)
2. object track (alogorithm developed in this study)
3. yield behavior identification
4. assess pedestrian safety based on identified yield behaviors

QUICK START (Ubuntu)

Install Oracle Java 8 on Ubuntu.

$ sudo add-apt-repository ppa:webupd8team/java

$ sudo apt-get update

$ sudo apt-get install oracle-java8-installer

$ sudo apt-get install oracle-java8-set-default

$ java -version

Install Git on Ubuntu.

$ sudo apt-get install git-core

Download or clone the repository to a local directory.

$ cd <local directory>
  
$ git clone https://github.com/nogrady/pedyield

Install Maven on Ubuntu.

$ sudo apt install maven

Build our project using Maven.

$ cd pedyield/

$ mvn clean compile package

Run the built -jar-with-dependencies.jar file.

$ cd target/

$ java -cp pedyield-0.0.1-SNAPSHOT-jar-with-dependencies.jar dzhuang.pedyield.detector.pedyield_detector -ip <> -iv <> [-t1 <>] [-t2 <>]

PARAMS
Params 	Description
-ip 	required, input file path of the pedestrian data
-iv 	required, input file path of the vehicle data
-t1 	optional, the threshold of not yield
-t2 	optional, the threshold of yield

EXAMPLE
https://www.youtube.com/watch?v=dxOufqxr8Xk&list=PL-ASdWq_qY8CUzvSGfKFhmFiMiGKsVSgz

CONFIGURATION

TBA

CALIBRATION

TBA

ACKNOWLEDGE
We gratefully acknowledge the support of NVIDIA Corporation with the donation of the Titan X Pascal used for this research. 

QUESTIONS

wonstran@hotmail.com
