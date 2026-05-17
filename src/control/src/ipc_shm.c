#include "shm_interface.h"
#include <string.h>

static vms_shared_state_t g_shared_state;

vms_shared_state_t *vms_shm_get(void)
{
    return &g_shared_state;
}

void vms_shm_init(void)
{
    memset(&g_shared_state, 0, sizeof(g_shared_state));
    atomic_store(&g_shared_state.flight_mode, MODE_DISARMED);
    atomic_store(&g_shared_state.tilt_command, 0.0f);
    atomic_store(&g_shared_state.flap_command, 0.0f);
    atomic_store(&g_shared_state.safety_override, 0);
    atomic_store(&g_shared_state.pid_gains_updated, 0);
    atomic_store(&g_shared_state.rc_valid, 0);
    atomic_store(&g_shared_state.loop_counter, 0);
}
