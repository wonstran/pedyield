About

This is a research 

Quick Start (Ubuntu)

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

Params
Params 	Description
-ip 	required, input file path of the pedestrian data
-iv 	required, input file path of the vehicle data
-t1 	optional, the threshold of not yield
-t2 	optional, the threshold of yield
Configuration

TBA
Calibration

TBA
Questions & Comments

zhuangdi1990, gmail, com
