#ifndef GOPHERCAN_CONFIG_H
#define GOPHERCAN_CONFIG_H

// RX and TX buffer sizes (bytes)
#define RX_BUFFER_SIZE 32
#define TX_BUFFER_SIZE 32

// number of connected CAN buses (max 3)
#define NUM_OF_BUSSES 1

// if defined, GCAN tries to retransmit messages on their destination bus
//#define CAN_ROUTER

#define TARGET F4XX

// if defined, all CAN messages are accepted
//#define NO_FILTER

// if defined, 11-bit ID data messages are filtered out
//#define IGNORE_DATA

#endif
