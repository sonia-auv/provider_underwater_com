#ifndef MODEM_DATA_H
#define MODEM_DATA_H

#pragma pack(push, 1)

typedef struct
{
    uint8_t packetNumber;
    uint8_t packetId : 7;
    uint8_t endOfPacket : 1;
} Header_t;

typedef struct
{
    Header_t header;
    uint8_t killSwitchState : 4, missionSwitchState : 4;
    uint16_t depth;
    uint8_t missionId;
    uint8_t missionState;
    uint8_t torpedosState : 4, droppersState : 4;
} Modem_M64_t;

// union bytes_8
// {
//     Modem_M64_t raw_data;
//     uint64_t u64_data;
// };

#pragma pack(pop)

#endif //MODEM_DATA_H