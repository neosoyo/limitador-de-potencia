#include "shell_cmds.h"
#include "control.h"
#include "persistence.h"
#include <string.h>

int shell_update_team_name(const char *name)
{
    if (!name) {
        return -EINVAL;
    }
    k_mutex_lock(&g_config_mutex, K_FOREVER);
    strncpy(g_config.team_name, name, sizeof(g_config.team_name) - 1);
    g_config.team_name[sizeof(g_config.team_name) - 1] = '\0';
    int ret = persistence_save_config(&g_config);
    k_mutex_unlock(&g_config_mutex);
    return ret;
}

int shell_update_team_number(uint32_t number)
{
    k_mutex_lock(&g_config_mutex, K_FOREVER);
    g_config.team_number = number;
    int ret = persistence_save_config(&g_config);
    k_mutex_unlock(&g_config_mutex);
    return ret;
}

int shell_update_pin(const char *pin)
{
    if (!pin || strlen(pin) != 6) {
        return -EINVAL;
    }
    k_mutex_lock(&g_config_mutex, K_FOREVER);
    strncpy(g_config.PIN_code, pin, sizeof(g_config.PIN_code) - 1);
    g_config.PIN_code[sizeof(g_config.PIN_code) - 1] = '\0';
    int ret = persistence_save_config(&g_config);
    k_mutex_unlock(&g_config_mutex);
    return ret;
}
