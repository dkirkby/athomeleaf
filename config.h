#ifndef CONFIG_H
#define CONFIG_H

#define CAPABILITY_TEMP_FEEDBACK     (1<<0)
#define CAPABILITY_LIGHT_FEEDBACK    (1<<1)

typedef struct {
    unsigned short capabilities;   // a bitmask of user-selectable device capabilities
    unsigned short temperatureMax; // comfort zone upper limit (degF x 100)
    unsigned short temperatureMin; // comfort zone lower limit (degF x 100)
    byte networkID; // a short identifier that uniquely identifies us on our local network
} Config;

#endif
