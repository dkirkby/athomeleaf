// =====================================================================
// Wireless code shared between leaf nodes and the hub. None of the
// functions defined here declare any stack variables.
//
// Copyright (c) 2010 David Kirkby dkirkby@uci.edu
// =====================================================================
#include "wireless.h"

// Low-level Nordic routines
#include "mirf.h"

// Wireless packet formats
#include "packet.h"

// Serial Peripheral Interface (SPI) pin assignments
#define SPI_SSEL        10
#define SPI_MOSI        11
#define SPI_MISO        12
#define SPI_SCLK        13

// Dedicated Nordic interface pin assignments
#define NORDIC_CE        8
#define NORDIC_IRQ       7

// Nordic configuration parameters
#define RADIO_CHANNEL 0
#define NORDIC_ADDR_LEN 3
#define NORDIC_MAX_RETRIES 15

// Global status byte indicates if the nordic transceiver is alive (which
// does not require that anyone is listening)
byte nordicOK;

// Nordic Tx/Rx addresses: should not alternate 101010... or have only one
// level transition. First array element is least-significant byte.
byte idleAddress[NORDIC_ADDR_LEN] =   { 0xEE, 0xEE, 0xEE };
// The least-significant 2 bytes [0:1] of the config address will be set based
// on a leaf node's serial number in initNoridc()
byte configAddress[NORDIC_ADDR_LEN] = { 0xFF, 0xFF, 0x9A };
// The following addresses must have the same last two (most-significant) bytes
byte dataAddress[NORDIC_ADDR_LEN] =   { 0xF2, 0xF2, 0xF2 };
byte lamAddress[NORDIC_ADDR_LEN]=     { 0xC6, 0xF2, 0xF2 };
byte dumpAddress[NORDIC_ADDR_LEN]=    { 0xA7, 0xF2, 0xF2 };

// Locally shared globals
static byte _wirelessByte;

// =====================================================================
// Initializes the Nordic nRF24L01+ transciever. Sets the value of
// the global nordicOK byte to either 1 (true) or 0 (false) to indicate
// if the initialization was successful. If successful, the transceiver
// is left in receive mode with CE high. The transciever is initialized
// for hub / leaf operation based on the serial number provided.
// =====================================================================

void initNordic(unsigned long serialNumber) {
    
    // nordic wireless initialization
    Mirf.csnPin = SPI_SSEL;
    Mirf.cePin = NORDIC_CE;
    Mirf.init();
    
    // configure MCU pin connected to the nordic IRQ as an input with internal pull-up
    pinMode(NORDIC_IRQ,INPUT);
    digitalWrite(NORDIC_IRQ,HIGH);
    
    // Set RF channel to use: 2400 + (RADIO_CHANNEL=0-99) MHz
	Mirf.configRegister(RF_CH,RADIO_CHANNEL);

    // Select a data rate of 250 kbps (the lowest possible) and a transmit
    // power of 0dB (the largest possible) for maximum receiver sensitivity.
    // At this data rate, maintaining <1% saturation with 16 packets sent
    // per second requires a packet length < 250000/1600 = 156 bits.
    // With a 3-byte address and 1-byte CRC, the framing overhead is
    // 6*8+1=49 bits so the maximum payload size is 13*8 = 104 < 107.
    Mirf.configRegister(RF_SETUP,0x26);

    // Use the maximum number of retries (16) and pick one of 4 different
    // retransmit delays based on the low-order bits of the serial number:
    // 1250,1500,1750,2000us.
    Mirf.configRegister(SETUP_RETR,
        NORDIC_MAX_RETRIES | 0x40 | ((byte)(serialNumber & 3) << 4));
    
    // Use a 1-byte CRC which catches all error bursts that last for no
    // more than 8 bits (32us at 250 kbps) and catches 255/256 = 99.61%
    // of any longer error burst (assuming random packet bits?).
    // A 2-byte CRC would reduce the undetected error rate but increase
    // the packet length and therefore collision rate.

    //...this is set by a #define mirf_CONFIG in mirf.h...

    // Use 3-byte addressing to minimize the packet length
#if NORDIC_ADDR_LEN==3
    Mirf.configRegister(SETUP_AW,0x01);
#elif NORDIC_ADDR_LEN==4
    Mirf.configRegister(SETUP_AW,0x02);
#elif NORDIC_ADDR_LEN==5
    Mirf.configRegister(SETUP_AW,0x03);
#else
#error "NORDIC_ADDR_LEN must be 3, 4 or 5"
#endif

    // P0 listens for auto-acks. Address must match the Tx address
    // and is updated in sendNordic(). Unless we are expecting an
    // auto-ack, listen on an unsued address.
    Mirf.writeRegister(RX_ADDR_P0,idleAddress,NORDIC_ADDR_LEN);
    
    // Setup the other (non-ack) receiver pipelines
    if(IS_HUB(serialNumber)) {
        // P1 listens for DataPackets
        Mirf.writeRegister(RX_ADDR_P1,dataAddress,NORDIC_ADDR_LEN);
    	Mirf.configRegister(RX_PW_P1,sizeof(DataPacket));
    	// P2 listens for Look-at-Me (LAM) packets. We only write the
    	// least-significant byte [0] (MS bytes [1:2] shared with P1 and P3)
        Mirf.configRegister(RX_ADDR_P2,lamAddress[0]);
    	Mirf.configRegister(RX_PW_P2,sizeof(LookAtMe));
    	// P3 listens for BufferDump packets. We only write the least-
    	// sigficant byte [0] (MS bytes [1:2] shared with P1 and P2)
        Mirf.configRegister(RX_ADDR_P3,dumpAddress[0]);
    	Mirf.configRegister(RX_PW_P3,sizeof(BufferDump));
        // Using P3,P2,P1,P0
        Mirf.configRegister(EN_RXADDR,0x0f);
    }
    else {
        // Use a per-device config address based on our serial number
        configAddress[0] = (byte)(serialNumber & 0xff);
        configAddress[1] = (byte)((serialNumber >> 8) & 0xff);
        // P1 listens for Config packets
        Mirf.writeRegister(RX_ADDR_P1,configAddress,NORDIC_ADDR_LEN);
    	Mirf.configRegister(RX_PW_P1,sizeof(Config));
        // Using P1,P0
        Mirf.configRegister(EN_RXADDR,0x03);
    }

    // Read back the least-significant byte [0] of the auto-ack pipeline (P0)
    // address to check that we are really talking to a nordic transceiver.
    // Use this register since it is configured the same in a hub and leaf.
	Mirf.readRegister(RX_ADDR_P0,&_wirelessByte,1);
	if(_wirelessByte == idleAddress[0]) {
        nordicOK = 1;
        // Start receiver
        Mirf.powerUpRx();
        Mirf.flushRx();
	}
	else {
        nordicOK = 0;
	}
    
}

// =====================================================================
// Reads the next available payload from the Rx FIFO into the buffer
// provided and returns the pipeline number (0-5) that it came from.
// Returns 0xff if the transceiver was never successfully initialized
// by initNordic() or 0xf0 if there is no data available. Does not
// check that the input payloadSize matches what the pipeline is
// configured for so, in case data is received from an unexpected
// pipeline, the returned payload may be truncated or have trailing
// garbage bytes.
// =====================================================================

byte getNordic(byte *payload, byte payloadSize) {

    if(!nordicOK) return 0xff;

    // Read our status register to check for a new packet and, if there
    // is one, find out which pipeline it came through.
//    Mirf.readRegister(STATUS,&_wirelessByte,1);
//    Serial.print(_wirelessByte,BIN);
//    Serial.write(' ');
    Mirf.readRegister(FIFO_STATUS,&_wirelessByte,1);
//    Serial.println(_wirelessByte,BIN);
//    delay(500);
    
    // Is there any data ready in our Rx FIFO?
    //Mirf.readRegister(FIFO_STATUS,&_wirelessByte,1);
    if(_wirelessByte & (1 << RX_EMPTY)) {
        return 0xf0;
    }
    
    // Which pipeline is the next payload from? This will be our
    // return value.
    Mirf.readRegister(STATUS,&_wirelessByte,1);
    _wirelessByte = (_wirelessByte >> RX_P_NO) & 0x07;
    
    // Read the payload from the pipeline
    Mirf.csnLow();
    Spi.transfer(R_RX_PAYLOAD);
    Mirf.transferSync(payload,payload,payloadSize);
    Mirf.csnHi();

    // Reset the RX_DR interrupt bit in the status register
    Mirf.configRegister(STATUS,(1<<RX_DR));
    
    return _wirelessByte;
}

// =====================================================================
// Sends the specified payload data synchronously to the specified
// address. Returns the number of retransmits necessary (ideally zero).
// If the return value is greater than 0x0f then the packet was lost.
// Returns 0xff in case the nordic transceiver was never successfully
// initialized by initNordic().
// =====================================================================

byte sendNordic(byte *address, byte *payload, byte payloadSize) {

    if(!nordicOK)  return 0xff;
    
    // Wait until any previous transmit has completed. In case the previous
    // transmit failed, its payload will still be in the Tx FIFO and we
    // could try re-sending it by toggling CE. We don't do this and instead
    // always flush the Tx FIFO below.
    //
    // This really isn't necessary as long as all data is sent using this
    // function, since it ends with a call to powerUpRx() that sets PTX=0
    while (Mirf.PTX) {
        Mirf.readRegister(STATUS,&_wirelessByte,1);
	    if((_wirelessByte & ((1 << TX_DS)  | (1 << MAX_RT)))){
		    Mirf.PTX = 0;
		    break;
	    }
    }

    // Disable the receiver and transmitter while we reconfigure
    Mirf.ceLow();

    // Configure pipeline-0 to receive an auto-ack from the receiver
    Mirf.writeRegister(RX_ADDR_P0,address,NORDIC_ADDR_LEN);
    
    // Set our transmit address
    Mirf.writeRegister(TX_ADDR,address,NORDIC_ADDR_LEN);

    // Switch to Tx mode and power up
    Mirf.powerUpTx();
    
    // Flush the Tx FIFO (in case the last transmit reached its max retries, it
    // will still be in the FIFO)
    Mirf.csnLow();
    Spi.transfer(FLUSH_TX);
    Mirf.csnHi();
    
    // Write this payload
    Mirf.csnLow();
    Spi.transfer(W_TX_PAYLOAD);
    Mirf.transmitSync(payload,payloadSize);
    Mirf.csnHi();

    // Start the transmission
    Mirf.ceHi();

    // Wait until the transmission is complete or fails
    Mirf.readRegister(STATUS,&_wirelessByte,1);
    // does this loop need a timeout? (no problems so far)
    while((_wirelessByte & (byte)((1 << MAX_RT)|(1 << TX_DS))) == 0) {
        Mirf.readRegister(STATUS,&_wirelessByte,1);
    }
    
    // Disable the receiver and transmitter while we cleanup
    Mirf.ceLow();

    // Fetch the contents of the transmit observe register, which gives
    // statistics on retransmissions. We will return this at the end.
    Mirf.readRegister(OBSERVE_TX,(byte*)&_wirelessByte,1);
    
    // In case we dropped any packets, reset the counter now.
    if(_wirelessByte & 0xf0) {
        Mirf.configRegister(RF_CH,RADIO_CHANNEL);
    }
    
    // Reset the P0 (auto-ack) address
    Mirf.writeRegister(RX_ADDR_P0,idleAddress,NORDIC_ADDR_LEN);
    
    // return to Rx mode
    Mirf.powerUpRx();
    
    return _wirelessByte;
}
