#include <stdio.h>

#include "emulator.h"

int main (int argc, char **argv)
{
    emulator_t *emulator = emulator_create ();

    emulator_load_bin (emulator, "test.bin");
    emulator_dump (emulator, stdout);

    uint8_t a = 5;
    uint8_t b = 7;

    uint8_t res = 0;

    uint8_t carry = 0;
    for (uint8_t i = 0; i < 8; ++i)
    {
        const uint8_t ba = a & _BV8 (i);
        const uint8_t bb = b & _BV8 (i);

        if (carry)
        {
            if (ba && bb)
            {
                res |= _BV8 (i);
                carry = true;
            }
            else if (ba || bb)
            {
                carry = true;
            }
            else
            {
                res |= _BV8 (i);
                carry = false;
            }
        }
        else
        {
            if (ba && bb)
            {
                carry = true;
            }
            else if (ba || bb)
            {
                res |= _BV8 (i);
            }
        }
    }

    printf ("%u\n", res);

    emulator_free (emulator);

    return 0;
}