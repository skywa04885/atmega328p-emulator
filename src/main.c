#include <stdio.h>

#include "emulator.h"

int main (int argc, char **argv)
{
    emulator_t *emulator = emulator_create ();

    emulator_load_bin (emulator, "test.bin.noignore");
    emulator_flash_program (emulator);

    m328p_reboot (emulator->m328p);
    m328p_run (emulator->m328p, 100000);

    emulator_dump (emulator, stdout);

    emulator_free (emulator);

    return 0;
}