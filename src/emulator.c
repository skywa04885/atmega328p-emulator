#include "emulator.h"

/// Creates an new emulator.
emulator_t *emulator_create (void)
{
    emulator_t *emulator = calloc (1, sizeof (emulator_t));
    if (emulator == NULL)
    {
        perror ("calloc () failed");
        return NULL;
    }

    emulator->m328p = m328p_create ();
    if (emulator->m328p == NULL)
    {
        free (emulator);
        return NULL;
    }

    return emulator;
}

/// Loads an binary program into the emulator.
emulator_err_t emulator_load_bin (emulator_t *emulator, const char *filename)
{
    FILE *fp = NULL;

    // Frees the existing program if there.
    if (emulator->program != NULL)
    {
        free (emulator->program);
    }

    // Opens the binary file.
    fp = fopen (filename, "r");
    if (fp == NULL)
    {
        perror ("fopen () failed");
        return EMULATOR_ERR_CANNOT_OPEN_FILE;
    }

    // Gets the size of the binary file.
    fseek (fp, (long) 0, SEEK_END);
    emulator->program_size = ftell (fp);
    fseek (fp, (long) 0, SEEK_SET);

    // Allocates the memory to contain the program.
    emulator->program = calloc (emulator->program_size, sizeof (uint8_t));
    if (emulator->program == NULL)
    {
        perror ("calloc () failed");
        return EMULATOR_ERR_NO_MEM;
    }

    // Reads the binary program, and closes the file.
    fread (emulator->program, sizeof (uint8_t), emulator->program_size, fp);
    fclose (fp);

    return EMULATOR_ERR_OK;
}

/// Runs n steps in the emulator and returns.
emulator_err_t emulator_run (emulator_t *emulator, int32_t n)
{
    return EMULATOR_ERR_OK;
}

/// Dumps the current emulator state.
emulator_err_t emulator_dump (emulator_t *emulator, FILE *fp)
{
    fprintf (fp, "-=- Emulator Dump (FaNcY) -=-\n");

    const uint8_t sreg_value = __m328p_read_sreg (emulator->m328p);
    const uint16_t spx_value = __m328p_read_spx (emulator->m328p);

    // Prints the values of the general registers.
    fprintf (fp, "Registers:\n");
    fprintf (fp, "\tSREG:\t%.6X hex, %.6u dec\n", (unsigned int) sreg_value, (unsigned int) sreg_value);
    fprintf (fp, "\tSPX:\t%.6X hex, %.6u dec\n", (unsigned int) spx_value, (unsigned int) spx_value);
    for (uint32_t reg = 0; reg < M328P_GEN_REG_COUNT; ++reg)
    {
        const uint8_t value = __m328p_read_general_register (emulator->m328p, reg);
        fprintf (fp, "\tR%u:\t%.6x hex, %.6u dec\n", reg, (unsigned int) value, (unsigned int) value);
    }

    return EMULATOR_ERR_OK;
}

/// Frees the given emulator.
void emulator_free (emulator_t *emulator)
{
    if (emulator->program != NULL)
    {
        free (emulator->program);
    }

    free (emulator->m328p);
    free (emulator);
}
