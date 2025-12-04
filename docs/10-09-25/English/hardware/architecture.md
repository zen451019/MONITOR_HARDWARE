# System Architecture
The system consists of a central controller responsible for data acquisition, processing, and transmission, supported by multiple transducers and conditioning modules that adapt the different signals for correct reading and interpretation by the controller.

![[../../res/img/hardware/System Architecture.svg]]

The raw signals undergo conditioning, are digitized by the controller, processed and classified, and finally serialized into data vectors for transmission via LoRaWAN. The gateway receives and forwards the data to the backend platform where it is stored and exploited.