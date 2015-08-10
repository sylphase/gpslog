#ifndef GUARD_VKPBQWSLJEMJEEOJ
#define GUARD_VKPBQWSLJEMJEEOJ

float measure_vdd();

bool hardware_get_battery_dead(float vdd);

bool hardware_get_battery_really_dead(float vdd);

void poweroff();

void set_led_color(double red, double green, double blue);

void hardware_init();

#endif
