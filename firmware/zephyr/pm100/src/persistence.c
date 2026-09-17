#include "persistence.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>


#if IS_ENABLED(CONFIG_SETTINGS)
#include <zephyr/settings/settings.h>

static bool config_loaded = false;
static struct device_config temp_config;

static int app_settings_set(const char *name, size_t len,
                            settings_read_cb read_cb, void *cb_arg)
{
    const char *next;
    if (settings_name_steq(name, "device", &next) && !next) {
        if (len == sizeof(struct device_config)) {
            read_cb(cb_arg, &temp_config, sizeof(struct device_config));
            config_loaded = true;
            return 0;
        }
    }
    return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(app, "app", NULL, app_settings_set, NULL, NULL);

int persistence_init(void)
{
    int ret = settings_subsys_init();
    if (ret) {
        LOG_ERR("Settings subsystem init failed (%d), falling back to RAM.", ret);
        return ret;
    }

    LOG_INF("Zephyr settings subsystem initialized successfully.");
    return 0;
}

int persistence_save_config(const struct device_config *cfg)
{
    int ret = settings_save_one("app/device", cfg, sizeof(struct device_config));
    if (ret < 0) {
        LOG_ERR("settings_save_one failed: %d", ret);
        return ret;
    }
    LOG_INF("Configuration successfully written to settings.");
    return 0;
}

int persistence_load_config(struct device_config *cfg)
{
    config_loaded = false;
    int ret = settings_load();
    if (ret) {
        LOG_WRN("settings_load failed: %d", ret);
        return ret;
    }
    if (config_loaded) {
        *cfg = temp_config;
        LOG_INF("Configuration successfully loaded from settings.");
        return 0;
    }
    LOG_WRN("Configuration not found in settings.");
    return -ENOENT;
}

#else /* IS_ENABLED(CONFIG_SETTINGS) == false */

// RAM-based simulated persistence fallback
static struct device_config ram_cache;
static bool ram_cache_valid = false;

int persistence_init(void)
{
    LOG_WRN("Settings subsystem not enabled in build (CONFIG_SETTINGS=n). Storing parameters in RAM.");
    return 0;
}

int persistence_save_config(const struct device_config *cfg)
{
    ram_cache = *cfg;
    ram_cache_valid = true;
    LOG_INF("Saved config in RAM cache (Settings subsystem disabled).");
    return 0;
}

int persistence_load_config(struct device_config *cfg)
{
    if (!ram_cache_valid) {
        LOG_INF("RAM persistence cache is uninitialized.");
        return -ENOENT;
    }
    *cfg = ram_cache;
    LOG_INF("Loaded config from RAM cache (Settings subsystem disabled).");
    return 0;
}

#endif
