#include "m328p.h"

///////////////////////////////////////////////////////////////////////////
// Atmega328P Subroutines
///////////////////////////////////////////////////////////////////////////

/// Creates an new Atmega328P.
m328p_t *m328p_create (void)
{
    return calloc ((size_t) 1, sizeof (m328p_t));
}

/// Frees the given Atmega328P.
void m328p_free (m328p_t *m328p)
{
    free (m328p);
}

/// Writes to an general register in the Atmega328P.
void __m328p_write_general_register (m328p_t *m328p, uint32_t n, uint8_t value)
{
    assert (n <= M328P_GEN_REG_MAX);

    m328p->sram[M328P_GEN_REG_OFFSET (n)] = value;
}

/// Reads from an general register in the Atmega328P.
uint8_t __m328p_read_general_register (m328p_t *m328p, uint32_t n)
{
    assert (n <= M328P_GEN_REG_MAX);    
    return m328p->sram[M328P_GEN_REG_OFFSET (n)];
}

/// Writes to an I/O Register.
void __m328p_write_io_register (m328p_t *m328p, uint32_t a, uint8_t value)
{
    assert (a <= M328P_IO_REG_MAX);
    m328p->sram[M328P_IO_REG_OFFSET (a)] = value;
}

/// Reads from an I/O Register.
uint8_t __m328p_read_io_register (m328p_t *m328p, uint32_t a)
{
    assert (a <= M328P_IO_REG_MAX);
    return m328p->sram[M328P_IO_REG_OFFSET (a)];
}

/// Reads the stack pointer.
uint16_t __m328p_read_spx (m328p_t *m328p)
{
    return (__m328p_read_io_register (m328p, M328P_SPL_IO_ADDR)
        | (__m328p_read_io_register (m328p, M328P_SPH_IO_ADDR) << 8));
}

///////////////////////////////////////////////////////////////////////////
// Atmega328P Instruction Subroutines
///////////////////////////////////////////////////////////////////////////

/// Executes the 'OUT' instruction.
m328p_err_t __m328p_instr_out (m328p_t *m328p, uint16_t opcode)
{
    if (!M328P_INSTR_OUT (opcode))
    {
        M328P_RETURN_ERR (M328P_ERR_INVALID_OPCODE);
    }

    const uint32_t A = ((opcode & 0b0000000000001111)
        | ((opcode & 0b0000011000000000) >> 9)); // Destination Address (I/O Space)
    const uint32_t Rr = ((opcode & 0b0000000111110000) >> 4); // Source Register


    const uint8_t Rr_value = __m328p_read_general_register (m328p, Rr);
    __m328p_write_io_register (m328p, A, Rr_value);

    m328p->cycles += M328P_INSTR_OUT_CYCLES;
    return M328P_ERR_OK;
}

/// Executes the 'IN' instruction.
m328p_err_t __m328p_instr_in (m328p_t *m328p, uint16_t opcode)
{
    if (!M328P_INSTR_IN (opcode))
    {
        M328P_RETURN_ERR (M328P_ERR_INVALID_OPCODE);
    }

    const uint32_t A = ((opcode & 0b0000000000001111)
        | ((opcode & 0b0000011000000000) >> 9)); // Destination Address (I/O Space)
    const uint32_t Rd = ((opcode & 0b0000000111110000) >> 4); // Destination Register

    const uint8_t A_value  = __m328p_read_io_register (m328p, A);
    __m328p_write_general_register (m328p, Rd, A_value);

    m328p->cycles += M328P_INSTR_IN_CYCLES;
    return M328P_ERR_OK;
}

/// Executes the 'JMP' instruction.
m328p_err_t __m328p_instr_jmp (m328p_t *m328p, uint16_t opcode, uint16_t extra)
{
    if (!M328P_INSTR_JMP (opcode))
    {
        M328P_RETURN_ERR (M328P_ERR_INVALID_OPCODE);
    }

    const uint32_t k =  ((uint32_t) extra)
        | ((opcode & 0b0000000111110000) << 5)
        | ((opcode & 0b0000000000000001) << 8); // Constant Address

    m328p->program_counter = k;
    m328p->cycles += M328P_INSTR_JMP_CYCLES;
    return M328P_ERR_OK;
}

/// Executes the 'ADD' and 'ADC' instruction.
m328p_err_t __m328p_instr_add_adc (m328p_t *m328p, uint16_t opcode, bool adc)
{
    // Reads the arguments.
    const uint8_t Rr = (((opcode & 0b0000001000000000) >> 5)
        | (opcode & 0b0000000000001111));
    const uint8_t Rd = (((opcode & 0b0000000100000000) >> 4)
        | ((opcode & 0b0000000011110000) >> 4));

    // Reads the value of the SREG register, so we can set and read some flags.
    uint8_t sreg_value = __m328p_read_sreg (m328p);

    // Reads the value of the source and destination registers.
    uint8_t Rr_value = __m328p_read_general_register (m328p, Rr),
        Rd_value = __m328p_read_general_register (m328p, Rd);

    // Performs the actual addition.
    bool carry = adc ? (sreg_value & M328P_SREG_C) : 0;
    uint8_t res = 0;

    for (uint8_t i = 0; i < 8; ++i)
    {
        const uint8_t ba = Rd_value & _BV8 (i);
        const uint8_t bb = Rr_value & _BV8 (i);

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

        // Set H if there was a carry from bit 3.
        if (carry && i == 3) sreg_value |= M328P_SREG_H;
        else sreg_value &= ~M328P_SREG_H;
    }

    // Set N if the MSB of the result is set.
    if (res & _BV8 (7)) sreg_value |= M328P_SREG_N;
    else sreg_value &= ~M328P_SREG_N;

    // Set Z if the result is zero.
    if (res == 0) sreg_value |= M328P_SREG_Z;
    else sreg_value &= ~M328P_SREG_Z;

    // Set the C flag if the carry flag remains set after addition.
    if (carry) sreg_value |= M328P_SREG_C;
    else sreg_value &= ~M328P_SREG_C;

    m328p->cycles += M328P_INSTR_ADC_ADD_CYCLES;
    return M328P_ERR_OK;
}
