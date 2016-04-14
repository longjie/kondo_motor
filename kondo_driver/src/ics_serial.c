/**
 * Kondo ICS device driver
 * @author Ryosuke Tajima
 */
#include <sys/select.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include "kondo_driver/ics_serial.h"

static int debug_level = 0;

int ics_set_debug_level(int level)
{
  debug_level = level;
}

/**
 * @brief open serial device
 * @param device device file name (i.e. "/dev/ttyUSB0")
 * @return file descripter of the ICS device
 */
int ics_open (const char* device)
{
  int fd = open (device, O_RDWR);
  if (fd < 0) {
    perror (device);
    return -1;
  }
  struct termios new;  
  memset(&new, 0, sizeof(new));
  /* set baud rate */
  cfsetospeed(&new, B115200);
  cfmakeraw(&new);
  new.c_cflag |= PARENB; // even parity bit
  // blocking mode
  new.c_cc[VTIME] = 0;
  new.c_cc[VMIN] = 1;
  int r = tcsetattr(fd, TCSAFLUSH, &new);
  if (r < 0) {
    perror ("tcsetattr");
  }
  return fd;
}

/**
 * @brief close device
 * @param file descripter of the ICS device
 */
void ics_close (int ics)
{
  close (ics);
}

/**
 * @brief send data to the device
 * @param buff data buffer to be sent
 * @param size size of the data (bytes)
 * @retval n>=0 sent size
 * @retval -1 error
 */
int ics_send (int ics, uint8_t* buff, size_t size)
{
  if (size > 68) {
    fprintf (stderr, "%s: size is too large.\n", __func__);
    return -1;
  }
  if (debug_level > 0) {
    for (int i=0; i<size; i++) {
      fprintf (stderr, "send: buff[%d]: %x\n", i, (unsigned int)buff[i]);
    }
  }
  int r = write(ics, buff, size);
  if (r != size) {
    fprintf (stderr, "%s: size mismatch.\n", __func__);
  }
  return r;
}

/**
 * @brief recieve data from the device
 * @param buff data buffer to be sent
 * @param timeout timeout[ms]
 * @return size of received data
 */
int ics_recv (int ics, uint8_t* buff, int size)
{
  int rsize, r;
  fd_set fds;
  struct timeval timeout;  
  timeout.tv_sec = 0;
  timeout.tv_usec = 20000;
  FD_ZERO(&fds);  
  FD_SET(ics, &fds);
  for (rsize=0; rsize < size; ) {
    r = select(ics+1, &fds, NULL, NULL, &timeout);
    if (r <= 0) {
      break;
    }
    r = read(ics, &buff[rsize], size-rsize);
    rsize += r;
  }
  if (debug_level > 0) {
    for (int i=0; i<rsize; i++) {
      fprintf (stderr, "recv: buff[%d]: %x\n", i, (unsigned int)buff[i]);
    }
  }
  return rsize;
}

/**
 * @brief synchronous communication to send then to recieve data
 * @param send_buff data buffer to be sent
 * @param send_size size of send_buff
 * @param recv_buff data buffer to be received
 * @param recv_size size of recv_buff
 * @retval 0 success
 * @retval -1 failure
 */
int ics_sync (int ics, uint8_t* send_buff, int send_size, uint8_t* recv_buff, int recv_size)
{
  int r;
  r = ics_send (ics, send_buff, send_size);
  if (r != send_size) {
    return -1;
  }
  r  = ics_recv (ics, recv_buff, recv_size);
  if (r != recv_size) {
    return -1;
  }
  return 0;
}

/**
 * @brief set command and recieve data from the device
 * @param buff data buffer to be sent
 * @retval 0 success
 * @retval -1 failure
 */
int ics_set_pulse (int ics, int id, uint16_t cmd_pulse, int* act_pulse)
{
  uint8_t sbuff[3], rbuff[6];
  sbuff[0] = id | ICS_CMD_POS;
  sbuff[1] = (cmd_pulse >> 7) & 0x7F; // high 7 bits of pulse
  sbuff[2] = cmd_pulse & 0x7F; // low 7 bits of pulse
  // fprintf (stderr, "cmd_pulse: %d\n", cmd_pulse);
  int r = ics_sync (ics, sbuff, sizeof(sbuff), rbuff, sizeof(rbuff));
  if (r < 0) {
    fprintf (stderr, "%s: ics_sync error.\n", __func__);
    return r;
  }
  if (act_pulse) {
    *act_pulse = ((rbuff[4] & 0x7F) << 7) | (rbuff[5] & 0x7F);
  }
  return 0;
}

int ics_set_hold (int ics, int id, int* pulse)
{
  return ics_set_pulse (ics, id, 16383, pulse);
}

int ics_set_free(int ics, uint8_t id, int* pulse)
{
  return ics_set_pulse (ics, id, 0, pulse);
}

int ics_set_stretch(int ics, uint8_t id, uint8_t value)
{
  char sbuff[3], rbuff[6];
  sbuff[0] = id | ICS_CMD_SET;
  sbuff[1] = ICS_SC_STRETCH;
  sbuff[2] = value;
  int r = ics_sync (ics, sbuff, sizeof(sbuff), rbuff, sizeof(rbuff));
  if (r < 0) {
    return -1;
  }
  return rbuff[5];
}

int ics_set_max_speed (int ics, uint8_t id, uint8_t value)
{
  char sbuff[3], rbuff[6];
  sbuff[0] = id | ICS_CMD_SET;
  sbuff[1] = ICS_SC_SPEED;
  sbuff[2] = value;
  int r = ics_sync (ics, sbuff, sizeof(sbuff), rbuff, sizeof(rbuff));
  if (r < 0) {
    return -1;
  }
  return rbuff[5];
}

int ics_set_max_current (int ics, uint8_t id, uint8_t value)
{
  char sbuff[3], rbuff[6];
  sbuff[0] = id | ICS_CMD_SET;
  sbuff[1] = ICS_SC_CURRENT;
  sbuff[2] = value;
  int r = ics_sync (ics, sbuff, sizeof(sbuff), rbuff, sizeof(rbuff));
  if (r < 0) {
    return -1;
  }
  return rbuff[5];
}

int ics_set_max_temperature (int ics, uint8_t id, uint8_t value)
{
  char sbuff[3], rbuff[6];
  sbuff[0] = id | ICS_CMD_SET;
  sbuff[1] = ICS_SC_TEMPERATURE;
  sbuff[2] = value;
  int r = ics_sync (ics, sbuff, sizeof(sbuff), rbuff, sizeof(rbuff));
  if (r < 0) {
    return -1;
  }
  return rbuff[5];
}

int ics_get_stretch(int ics, uint8_t id)
{
  char sbuff[2], rbuff[5];
  sbuff[0] = id | ICS_CMD_GET;
  sbuff[1] = ICS_SC_STRETCH;
  int r = ics_sync (ics, sbuff, sizeof(sbuff), rbuff, sizeof(rbuff));
  if (r < 0) {
    return -1;
  }
  return rbuff[4];
}


int ics_get_max_speed(int ics, uint8_t id)
{
  uint8_t sbuff[2], rbuff[5];
  sbuff[0] = id | ICS_CMD_GET;
  sbuff[1] = ICS_SC_SPEED;
  int r = ics_sync (ics, sbuff, sizeof(sbuff), rbuff, sizeof(rbuff));
  if (r < 0) {
    return -1;
  }
  return rbuff[4];
}

int ics_get_current (int ics, uint8_t id)
{
  char sbuff[2], rbuff[5];
  sbuff[0] = id | ICS_CMD_GET;
  sbuff[1] = ICS_SC_CURRENT;
  int r = ics_sync (ics, sbuff, sizeof(sbuff), rbuff, sizeof(rbuff));
  if (r < 0) {
    return -1;
  }
  /* 0 to 63 when CW, 64 to 127 when CCW */
  int current = rbuff[4];
  if (current >= 64) {
    current = 64 - current;
  }
  return current;
}

int ics_get_temperature (int ics, uint8_t id)
{
  char sbuff[2], rbuff[5];
  sbuff[0] = id | ICS_CMD_GET;
  sbuff[1] = ICS_SC_TEMPERATURE;
  int r = ics_sync (ics, sbuff, sizeof(sbuff), rbuff, sizeof(rbuff));
  if (r < 0) {
    return -1;
  }
  return rbuff[4];
}

int ics_get_id (int ics)
{
  int r;
  char sbuff[4], rbuff[5];
  sbuff[0] = 0xFF;
  sbuff[1] = ICS_SC_READ;
  sbuff[2] = ICS_SC_READ;
  sbuff[3] = ICS_SC_READ;
  r = ics_sync (ics, sbuff, sizeof(sbuff), rbuff, sizeof(rbuff));
  if (r < 0) {
    fprintf (stderr, "%s: ics_sync\n", __func__);
    return r;
  }
  /* somehow, get_id command needs big sleep before following commands */
  usleep (500000);
  return rbuff[4] & 0x1F;
}

int ics_set_id(int ics, int id)
{
  uint8_t sbuff[4], rbuff[5];
  sbuff[0] = id | ICS_CMD_ID;
  sbuff[1] = ICS_SC_WRITE;
  sbuff[2] = ICS_SC_WRITE;
  sbuff[3] = ICS_SC_WRITE;
  int r = ics_sync (ics, sbuff, sizeof(sbuff), rbuff, sizeof(rbuff));
  /* somehow, get_id command needs big sleep before following commands */
  usleep (500000);
  return rbuff[4] & 0x1F;
}

typedef struct {
  uint8_t backup_char[2];
  uint8_t stretch;
  uint8_t speed;
  uint8_t punch;
  uint8_t dead_band;
  uint8_t dumping;
  uint8_t safe_timer;
  uint8_t flag;
  uint16_t pulse_max;
  uint16_t pulse_min;
  uint8_t baud_rate;
  uint8_t max_temperature;
  uint8_t max_current;

} ics_eeprom_t;

int ics_get_eeprom(int ics, int id, uint8_t* eeprom)
{
  char sbuff[2], rbuff[68];
  sbuff[0] = id | ICS_CMD_GET;
  sbuff[1] = 0;
  int r = ics_sync (ics, sbuff, sizeof(sbuff), rbuff, sizeof(rbuff));
  if (r < 0) {
    return -1;
  }
  memcpy(eeprom, &rbuff[4], 64);
  return r;
}

int ics_set_eeprom(int ics, int id, uint8_t* eeprom)
{
  char sbuff[66], rbuff[68];
  sbuff[0] = id | ICS_CMD_SET;
  sbuff[1] = 0;
  memcpy(&sbuff[2], eeprom, 64);

  int r;
  r = ics_send (ics, sbuff, sizeof(sbuff));
  if (r != sizeof(sbuff)) {
    fprintf (stderr, "%s: send error.\n", __func__);
    return -1;
  }
  usleep (5000000);
  r = ics_recv (ics, rbuff, sizeof(rbuff));
  if (r != sizeof(rbuff)) {
    fprintf (stderr, "%s: recv error.\n", __func__);
    return -1;
  }
#if 0
  int r = ics_sync (ics, sbuff, sizeof(sbuff), rbuff, sizeof(rbuff));
  if (r < 0) {
    fprintf (stderr, "%s: ics_sync error.\n", __func__);
    return -1;
  }
#endif
  /* check sbuff matches the echo back */
  for (int i=0; i<66; i++) {
    if (sbuff[i] != rbuff[i]) {
      fprintf (stderr, "%s: doesn't match the echo back.\n", __func__);
      return -1;
    }
  }
  return 0;
}

int ics_eeprom_set_stretch(uint8_t* eeprom, uint8_t value)
{
  eeprom[2] = value >> 4;
  eeprom[3] = value & 0x0f;
}

uint8_t ics_eeprom_stretch(uint8_t* eeprom)
{
  return (eeprom[2] << 4) | (eeprom[3] & 0x0f);
}

uint8_t ics_eeprom_speed(uint8_t* eeprom)
{
  return (eeprom[4] << 4) | (eeprom[5] & 0x0f);
}

uint8_t ics_eeprom_punch(uint8_t* eeprom)
{
  return (eeprom[6] << 4) | (eeprom[7] & 0x0f);
}

uint8_t ics_eeprom_dead_band(uint8_t* eeprom)
{
  return (eeprom[8] << 4) | (eeprom[9] & 0x0f);
}

uint8_t ics_eeprom_dumping(uint8_t* eeprom)
{
  return (eeprom[10] << 4) | (eeprom[11] & 0x0f);
}

uint8_t ics_eeprom_safe_timer(uint8_t* eeprom)
{
  return (eeprom[12] << 4) | (eeprom[13] & 0x0f);
}

uint8_t ics_eeprom_flag (uint8_t* eeprom)
{
  return (eeprom[14] << 4) | (eeprom[15] & 0x0f);
}

uint16_t ics_eeprom_max_pulse (uint8_t* eeprom)
{
  return ((eeprom[16] & 0x0f) << 12 | (eeprom[17] & 0x0f) << 8 |
          (eeprom[18] & 0x0f) << 4  | (eeprom[19] & 0x0f));
}

uint16_t ics_eeprom_min_pulse (uint8_t* eeprom)
{
  return ((eeprom[20] & 0x0f) << 12 | (eeprom[21] & 0x0f) << 8 |
          (eeprom[22] & 0x0f) << 4  | (eeprom[23] & 0x0f));
}

uint8_t ics_eeprom_baud_rate (uint8_t* eeprom)
{
  return (eeprom[26] & 0x0f) << 4 | (eeprom[27] & 0x0f);
}

uint8_t ics_eeprom_max_temperature (uint8_t* eeprom)
{
  return (eeprom[28] & 0x0f) << 4 | (eeprom[29] & 0x0f);
}

uint8_t ics_eeprom_max_current (uint8_t* eeprom)
{
  return (eeprom[30] & 0x0f) << 4 | (eeprom[31] & 0x0f);
}

uint8_t ics_eeprom_response (uint8_t* eeprom)
{
  return (eeprom[50] & 0x0f) << 4 | (eeprom[51] & 0x0f);
}

uint8_t ics_eeprom_user_offset (uint8_t* eeprom)
{
  return (eeprom[52] & 0x0f) << 4 | (eeprom[53] & 0x0f);
}

uint8_t ics_eeprom_id (uint8_t* eeprom)
{
  return (eeprom[56] & 0x0f) << 4 | (eeprom[57] & 0x0f);
}

uint8_t ics_eeprom_characteristic_stretch_1 (uint8_t* eeprom)
{
  return (eeprom[58] & 0x0f) << 4 | (eeprom[59] & 0x0f);
}

uint8_t ics_eeprom_characteristic_stretch_2 (uint8_t* eeprom)
{
  return (eeprom[60] & 0x0f) << 4 | (eeprom[61] & 0x0f);
}

uint8_t ics_eeprom_characteristic_stretch_3 (uint8_t* eeprom)
{
  return (eeprom[62] & 0x0f) << 4 | (eeprom[63] & 0x0f);
}
