/**
 * @brief command to get/set ICS id.
 *
 * make sure you connect only one servo motor to the interface.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>
#include "kondo_driver/ics_serial.h"

char g_device[256] = "/dev/ttyUSB0";
int g_id = -1;

/**
 * @brief parse command line options
 */
int parse_options(int argc, char** argv)
{
  int opt, index;
  struct option longopt[] = {
    {"device", optional_argument, 0, 'd' },
    {"id",     optional_argument, 0, 'i' },
    {0,        0,                 0,  0  }
  };
  while ((opt = getopt_long(argc, argv, "d:i:", longopt, &index)) != -1) {
    switch (opt) {
      case 'd':
        strncpy(g_device, optarg, sizeof(g_device));
        break;
      case 'i':
        g_id = atoi(optarg);
        break;
      default:
        fprintf(stderr, "Usage: %s [-d device] [-i id]\n", argv[0]);
        exit(1);
    }
  }
  return 0;
}

/**
 * @brief main function
 */
int main(int argc, char **argv)
{
  // parse command line options
  parse_options(argc, argv);

  int ics_fd = ics_open (g_device);
  if (ics_fd < 0) {
    perror ("open");
    exit (1);
  }
  int id = ics_get_id (ics_fd);
  fprintf (stderr, "Current servo ID: %d\n", id);

  if (g_id >= 0) {
    // Set servo Id
    int r = ics_set_id (ics_fd, g_id);
    if (g_id != r) {
	fprintf (stderr, "Could not set ID.\n");
	exit(0);
    }
    sleep (3);
    fprintf (stderr, "Set servo ID correctly: %d\n", g_id);
    r = ics_get_id (ics_fd);
    if (g_id != r) {
      fprintf (stderr, "Set servo ID correctly: %d\n", g_id);
    }
  }
  return 0;
}	
