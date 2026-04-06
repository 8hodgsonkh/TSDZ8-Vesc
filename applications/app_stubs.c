/*
 * app_stubs.c — Stub implementations for unused app modules (PPM, Nunchuk)
 * These are not used on this hardware but are referenced by app.c, commands.c,
 * and lispif_vesc_extensions.c, so they need stub symbols.
 */

#include "app.h"
#include "datatypes.h"

// ============ app_ppm stubs ============

void app_ppm_start(void) {}
void app_ppm_stop(void) {}
float app_ppm_get_decoded_level(void) { return 0.0f; }
void app_ppm_detach(bool detach) { (void)detach; }
void app_ppm_override(float val) { (void)val; }
void app_ppm_configure(ppm_config *conf) { (void)conf; }

// ============ app_nunchuk stubs ============

void app_nunchuk_start(void) {}
void app_nunchuk_stop(void) {}
void app_nunchuk_configure(chuk_config *conf) { (void)conf; }
float app_nunchuk_get_decoded_x(void) { return 0.0f; }
float app_nunchuk_get_decoded_y(void) { return 0.0f; }
bool app_nunchuk_get_bt_c(void) { return false; }
bool app_nunchuk_get_bt_z(void) { return false; }
bool app_nunchuk_get_is_rev(void) { return false; }
float app_nunchuk_get_update_age(void) { return 0.0f; }
void app_nunchuk_update_output(chuck_data *data) { (void)data; }
