#include <stdio.h>
#include <gtest/gtest.h>
extern "C" {
#include "kondo_driver/ics_serial.h"
}

int ics;

TEST(ICSTest, open)
{
  ics = ics_open ("/dev/ttyUSB0");
  EXPECT_FALSE (ics < 0);
  ics_set_debug_level(1);
}

TEST(ICSTest, set_and_get_id)
{
  int id;
  /* set id */
  id = ics_set_id (ics, 16);
  EXPECT_EQ(id, 16);
  /* get id */
  id = ics_get_id (ics);
  EXPECT_EQ(id, 16);
  /* set id */
  id = ics_set_id (ics, 0);
  EXPECT_EQ(id, 0);
  /* get id */
  id = ics_get_id (ics);
  EXPECT_EQ(id, 0);
}

TEST(ICSTest, set_pulse)
{
  int r;
  int act_pos;
  /* set hold */
  r = ics_set_pulse (ics, 0, 7500, &act_pos);
  EXPECT_EQ(r, 0);
  /* set hold */
  r = ics_set_pulse (ics, 0, 7300, &act_pos);
  EXPECT_EQ(r, 0);
  /* set hold */
  r = ics_set_pulse (ics, 0, 7700, &act_pos);
  EXPECT_EQ(r, 0);
  /* unconnected device */
  r = ics_set_pulse (ics, 1, 7500, &act_pos);
  EXPECT_TRUE(r < 0);
}

#if 0
TEST(ICSTest, set_hold)
{
  int id = 0;
  int act_pos;
  int r = ics_set_hold(ics, 0, &act_pos);
  EXPECT_EQ(r, 0);
}
#endif

TEST(ICSTest, set_free)
{
  int id = 0;
  int act_pos;
  int r = ics_set_free(ics, id, &act_pos);
  EXPECT_EQ(r, 0);
}

TEST(ICSTest, stretch)
{
  int value;
  value = ics_set_stretch(ics, 0, 15);
  EXPECT_EQ(value, 15);
  value = ics_get_stretch(ics, 0);
  EXPECT_EQ(value, 15);
  value = ics_set_stretch(ics, 0, 30);
  EXPECT_EQ(value, 30);
}

TEST(ICSTest, max_speed)
{
  int value;
  //value = ics_set_max_speed(ics, 0, 128);
  //EXPECT_EQ(value, 128);
  //value = ics_get_max_speed(ics, 0);
  //EXPECT_EQ(value, 128);
}

TEST(ICSTest, max_current)
{
  int value;
  value = ics_get_current(ics, 0);
  EXPECT_TRUE(abs(value) < 2);
}

uint8_t eeprom[64];

TEST(ICSTest, get_eeprom)
{
  int r = ics_get_eeprom(ics, 0, eeprom);
  EXPECT_EQ(r, 0);
  int value;
  value = ics_eeprom_stretch (eeprom);
  EXPECT_EQ(value, 60);
  value = ics_eeprom_speed (eeprom);
  EXPECT_EQ(value, 128); // why 128? should be 127
  value = ics_eeprom_punch (eeprom);
  EXPECT_EQ(value, 1);
  value = ics_eeprom_dead_band (eeprom);
  EXPECT_EQ(value, 2);
  value = ics_eeprom_dumping (eeprom);
  EXPECT_EQ(value, 40);
  value = ics_eeprom_safe_timer (eeprom);
  EXPECT_EQ(value, 250);
  value = ics_eeprom_flag (eeprom);
  EXPECT_FALSE(value & ICS_FLAG_REVERSE);
  EXPECT_TRUE(value & ICS_FLAG_FREE);
  EXPECT_FALSE(value & ICS_FLAG_PWMINH);
  EXPECT_TRUE(value & ICS_FLAG_INFREV);
  EXPECT_FALSE(value & ICS_FLAG_SLAVE);
  /* pulse limits */
  value = ics_eeprom_max_pulse (eeprom);
  EXPECT_EQ(value, 11500);
  value = ics_eeprom_min_pulse (eeprom);
  EXPECT_EQ(value, 3500);
  /* baud rate */
  value = ics_eeprom_baud_rate (eeprom);
  EXPECT_EQ(value, 10);
  value = ics_eeprom_max_temperature (eeprom);
  EXPECT_EQ(value, 80);
  value = ics_eeprom_max_current (eeprom);
  EXPECT_EQ(value, 63);
  value = ics_eeprom_response (eeprom);
  EXPECT_EQ(value, 3);
  value = ics_eeprom_user_offset (eeprom);
  EXPECT_EQ(value, 0);
  value = ics_eeprom_id (eeprom);
  EXPECT_EQ(value, 0);
  /* characteristic streatch */
  value = ics_eeprom_characteristic_stretch_1 (eeprom);
  EXPECT_EQ(value, 120);
  value = ics_eeprom_characteristic_stretch_2 (eeprom);
  EXPECT_EQ(value, 60);
  value = ics_eeprom_characteristic_stretch_3 (eeprom);
  EXPECT_EQ(value, 254);
}

TEST(ICSTest, set_eeprom)
{
  ics_eeprom_set_stretch(eeprom, 60);
  int r = ics_set_eeprom(ics, 0, eeprom);
  EXPECT_EQ(r, 0);
}

int main (int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}
