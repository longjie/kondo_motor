#include <stdint.h>

#define ICS_CMD_POS 0x80
#define ICS_CMD_GET 0xA0
#define ICS_CMD_SET 0xC0
#define ICS_CMD_ID 0xE0
#define ICS_SC_EEPROM  0
#define ICS_SC_STRETCH 1
#define ICS_SC_SPEED   2
#define ICS_SC_CURRENT 3
#define ICS_SC_TEMPERATURE 4
#define ICS_SC_READ 0
#define ICS_SC_WRITE 1

#define ICS_FLAG_REVERSE (0x01)
#define ICS_FLAG_FREE    (0x02)
#define ICS_FLAG_PCM     (0x04)
#define ICS_FLAG_PWMINH  (0x08)
#define ICS_FLAG_INFREV  (0x10)
#define ICS_FLAG_SLAVE   (0x80)

int ics_open (const char* device);
void ics_close (int ics);
int ics_sync (int ics, uint8_t* send_buff, int send_size, uint8_t* recv_buff, int recv_size);
int ics_get_id (int ics);
int ics_set_id(int ics, int id);
int ics_set_pulse (int ics, int id, uint16_t cmd_pulse, int* act_pulse);
int ics_set_hold (int ics, int id, int* pos);

int ics_set_stretch(int ics, uint8_t id, uint8_t value);
int ics_set_max_speed (int ics, uint8_t id, uint8_t value);
int ics_set_max_current (int ics, uint8_t id, uint8_t value);
int ics_set_max_temperature (int ics, uint8_t id, uint8_t value);

int ics_set_free(int ics, uint8_t id, int* pos);
int ics_get_stretch(int ics, uint8_t id);
int ics_get_max_speed(int ics, uint8_t id);
int ics_get_current (int ics, uint8_t id);
int ics_get_temperature (int ics, uint8_t id);

int ics_set_eeprom(int ics, int id, uint8_t* eeprom);
int ics_get_eeprom(int ics, int id, uint8_t* eeprom);
uint8_t ics_eeprom_stretch(uint8_t* eeprom);
uint8_t ics_eeprom_speed(uint8_t* eeprom);
uint8_t ics_eeprom_punch(uint8_t* eeprom);
uint8_t ics_eeprom_dead_band(uint8_t* eeprom);
uint8_t ics_eeprom_dumping(uint8_t* eeprom);
uint8_t ics_eeprom_safe_timer(uint8_t* eeprom);
uint8_t ics_eeprom_flag (uint8_t* eeprom);
uint16_t ics_eeprom_max_pulse (uint8_t* eeprom);
uint16_t ics_eeprom_min_pulse (uint8_t* eeprom);
uint8_t ics_eeprom_baud_rate (uint8_t* eeprom);
uint8_t ics_eeprom_max_temperature (uint8_t* eeprom);
uint8_t ics_eeprom_max_current (uint8_t* eeprom);
uint8_t ics_eeprom_response (uint8_t* eeprom);
uint8_t ics_eeprom_user_offset (uint8_t* eeprom);
uint8_t ics_eeprom_id (uint8_t* eeprom);
uint8_t ics_eeprom_characteristic_stretch_1 (uint8_t* eeprom);
uint8_t ics_eeprom_characteristic_stretch_2 (uint8_t* eeprom);
uint8_t ics_eeprom_characteristic_stretch_3 (uint8_t* eeprom);

int ics_eeprom_set_stretch(uint8_t* eeprom, uint8_t value);

int ics_set_debug_level(int level);
