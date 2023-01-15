
# Team MakeItReal

We are a team of Polisch highschool students participating in the 2022/2023 CanSat competition. The goal of our secondary mission is to build establishing two-way communication between the network and the ground base, we will commission it to perform various tasks, e.g., uploading and modifying a file to each micro-sat, and then returning it to the base, or sending temperature and pressure data from each micro-sat using a radio connection only with one of them.    


## Links

 - [Instagram](https://www.instagram.com/makeitreal__team/)
 - [Support our project!](https://zrzutka.pl/untfp4)




## Mission Overview

Our CanSat will be prepared to fall from an attitude between about 500-3000 metres from a drone or a
rocket. As soon as the CanSat starts descending 3 parachutes will deploy. This will allow our CanSat to split
into 3 microsatellites. Each capable of collecting, sending. Every Part of the satellite will have an onboard data collection system which consists of a pressure and
temperature sensor. Our microcontroller will be responsible for collecting data from the sensors, recording
them on an SD card, and sending via the radio module.
Every CanSat needs to land slowly to assure the safety of the mission. After landing, communication
between the microsatellites will start. This will allow us to create a small data network capable of collecting
data from many places and then sending them to the one that’s connected to the base. The system is
optimised to collect and transfer data every second.
The base transmitter will stop the connection with the transmitters and switch to network mode. This will
be achieved by sending a ping signal and the first part to receive it (the closest one) will start a 2-way
communication with the part that’s closest to the base. This will be possible thanks to the directional
antenna at the station and onboard LoRa modules.
Using the collected data we will be able to draw conclusions about the environment we are testing.
Additionally to what was said above, we plan to calculate the signal delay of the satellite that is closest to
the base. It will allow us to know the approximate distance from the microsatellite without using GPS. We
know that it is physically possible but don’t know if we have the resources to achieve it.


![Logo](https://zrzutka.pl/uploads/chipin/untfp4/cover/orginal/98900a958685e8c98d36d2553ba06ab4.jpg)

