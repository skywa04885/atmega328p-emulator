#pragma once

#include <stdio.h>

#include "m328p.h"

typedef struct
{
    m328p_t     *m328p;
    // --- //
    uint8_t     *program;
    size_t      program_size;
} emulator_t;

typedef enum
{
    EMULATOR_ERR_OK,
    EMULATOR_ERR_NO_MEM,
    EMULATOR_ERR_M328P,
    EMULATOR_ERR_CANNOT_OPEN_FILE
} emulator_err_t;

/// Creates an new emulator.
emulator_t *emulator_create (void);

/// Loads an binary program into the emulator.
emulator_err_t emulator_load_bin (emulator_t *emulator, const char *filename);

/// Flashes the Atmega328P in the emulator.
emulator_err_t emulator_flash_program (emulator_t *emulator);

/// Dumps the current emulator state.
emulator_err_t emulator_dump (emulator_t *emulator, FILE *fp);

/// Frees the given emulator.
void emulator_free (emulator_t *emulator);
