#ifndef GUARD_VKPBQWSLJEMJEEOJ
#define GUARD_VKPBQWSLJEMJEEOJ

enum class LEDColor {
    OFF,
    RED,
    GREEN,
    BLUE,
};
void set_led_color(LEDColor x);

float measure_vdd();

bool hardware_get_battery_dead();

bool hardware_get_battery_really_dead();

void poweroff();

void hardware_init();

#endif
