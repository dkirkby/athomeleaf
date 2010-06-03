#ifndef CONFIG_H
#define CONFIG_H

#define CONFIG_HEADER 0xDeadBeef

#define CAPABILITY_TEMP_FEEDBACK     (1<<0)
#define CAPABILITY_LIGHT_FEEDBACK    (1<<1)
#define CAPABILITY_LIGHT_DUMP        (1<<2)
#define CAPABILITY_POWER_DUMP        (1<<3)

typedef struct {
    unsigned long header; // a fixed header to help filter spurious config packets
    byte networkID; // a short identifier that uniquely identifies us on our local network
    byte capabilities; // a bitmask of user-selectable device capabilities
    unsigned short temperatureMin; // comfort zone upper limit (degF x 100)
    unsigned short temperatureMax; // comfort zone lower limit (degF x 100)
} Config;

#endif
