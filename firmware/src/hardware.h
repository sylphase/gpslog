#ifndef GUARD_VKPBQWSLJEMJEEOJ
#define GUARD_VKPBQWSLJEMJEEOJ

float measure_vdd();

bool hardware_get_battery_dead(float vdd);

bool hardware_get_battery_really_dead(float vdd);

void poweroff();

void set_led_color(float red, float green, float blue);
void set_led_override_off(bool override_off);

void hardware_init();

#if defined SYLPHASE_GPSLOG_2A
void set_external_led(bool on);
#endif

#endif
