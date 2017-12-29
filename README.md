# About PFB channelizer
This design is based on the Harris and McGwier implementation of a multichannel filter bank with multiple channel bandwidths that can be found at [1], and is a demostration on how to use channelization using the Ettus Research's RFNoC framework.

## Basic Dependencies
* UHD >= 3.10.1.0
* GNU Radio >= 3.7.10.1

## Installation

Clone this repository in your local machine:\
    `$ git clone https://github.com/EttusResearch/rfnoc-pfb-channelizer.git`\
Follow the common steps for build and install\
    `$ cd rfnoc-pfb-channelizer`\
    `$ mkdir build`\
    `$ cd build`\
    `$ cmake ..`\
    `$ make`\
    `$ make test`\
    `$ sudo make install`\
    `$ sudo ldconfig`

## Documentation
* A white paper that explains the theory and implementation of the channelizer can be found in docs/Channelizer.pdf
* A paper was published in the GNU Radio procedings:
VALLANCE, Phillip. Channelization using RFNoC. Proceedings of the GNU Radio Conference, [S.l.], v. 2, n. 1, p. 7, sep. 2017. Available at: <https://pubs.gnuradio.org/index.php/grcon/article/view/18>. Date accessed: 5 dec. 2017
* This implementation was presented during GRCon17: https://youtu.be/DVBgLuzUlRQ

# References
[1] Harris,  F.,  McGwier  R.,  Egg  B.  "A  Versatile  Multichannel  Filter  Bank  with  MultipleChannel Bandwidths"June 2010. CrownCOM 5th International Conference on CognitiveRadio Oriented Wireless Networks and Communications

