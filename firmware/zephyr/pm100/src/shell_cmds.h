#ifndef SHELL_CMDS_H
#define SHELL_CMDS_H

#include <zephyr/kernel.h>

/**
 * @brief Thread-safe configuration update helper for the team name.
 * 
 * Configures the team name string and immediately persists it to NVS storage.
 * 
 * @param name Team name string.
 * @return int 0 on success, or a negative error code on failure.
 */
int shell_update_team_name(const char *name);

/**
 * @brief Thread-safe configuration update helper for the team number.
 * 
 * Configures the team index number and immediately persists it to NVS storage.
 * 
 * @param number Team index number.
 * @return int 0 on success, or a negative error code on failure.
 */
int shell_update_team_number(uint32_t number);

/**
 * @brief Thread-safe configuration update helper for the security PIN code.
 * 
 * Validates and configures the 4-digit PIN, immediately persisting it to storage.
 * 
 * @param pin 4-character PIN code string.
 * @return int 0 on success, or a negative error code on failure.
 */
int shell_update_pin(const char *pin);

#endif /* SHELL_CMDS_H */
