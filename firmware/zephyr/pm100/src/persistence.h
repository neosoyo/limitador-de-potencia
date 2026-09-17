#ifndef __PERSISTENCE_H
#define __PERSISTENCE_H

#include "control.h"

/**
 * @brief Initialize the persistence layer.
 * 
 * Attempts to mount Zephyr's NVS partition. If NVS is not compiled in or fails,
 * it falls back to simulated persistence in RAM.
 * 
 * @return int 0 on success, or a negative error code on failure.
 */
int persistence_init(void);

/**
 * @brief Save the device configuration struct.
 * 
 * Writes the configuration struct to NVS storage (ID 1) or updates the RAM cache.
 * 
 * @param cfg Pointer to the device_config structure to save.
 * @return int 0 on success, or a negative error code on failure.
 */
int persistence_save_config(const struct device_config *cfg);

/**
 * @brief Load the device configuration struct.
 * 
 * Reads the configuration struct from NVS storage (ID 1) or from the RAM cache.
 * 
 * @param cfg Pointer to the device_config structure to load into.
 * @return int 0 on success, or a negative error code on failure.
 */
int persistence_load_config(struct device_config *cfg);

#endif /* __PERSISTENCE_H */
