#ifndef GUARD_DZQMKRJSEKQLKCNT
#define GUARD_DZQMKRJSEKQLKCNT

void gps_setup();
void gps_start_logging();
bool gps_write_stamped_packet(uint8_t const * data, uint32_t length);

#endif
