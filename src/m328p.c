#include "m328p.h"

///////////////////////////////////////////////////////////////////////////
// Atmega328P Subroutines
///////////////////////////////////////////////////////////////////////////

/// Creates an new Atmega328P.
m328p_t *m328p_create (void)
{
    return calloc ((size_t) 1, sizeof (m328p_t));
}

/// Flashes the given program onto the Atmega328P.
m328p_err_t m328p_flash (m328p_t *m328p, uint8_t *program, size_t program_size)
{
    if (program_size > M328P_FLASH)
    {
        M328P_RETURN_ERR (M328P_ERR_NOT_ENOUGH_FLASH);   
    }

    memcpy (m328p->flash, program, program_size);
    return M328P_ERR_OK;
}

/// Reboots the Atmega328P.
m328p_err_t m328p_reboot (m328p_t *m328p)
{
    // Resets the program counter and the number of cycles.
    m328p->program_counter = 0x0000;
    m328p->cycles = 0;

    // Clears the SRAM.
    memset (m328p->sram, 0, M328P_SRAM);

    return M328P_ERR_OK;
}

/// Runs the Atmega328P for n cycles.
m328p_err_t m328p_run (m328p_t *m328p, size_t cycles)
{
    size_t exit_at = m328p->cycles + cycles;
    while (m328p->cycles <= exit_at)
    {
        m328p_err_t err;
        if ((err = m328p_exec_single_instruction (m328p)) != M328P_ERR_OK) return err;
    }

    return M328P_ERR_OK;
}

/// Executes an single instruction on the Atmega328P.
m328p_err_t m328p_exec_single_instruction (m328p_t *m328p)
{
    const uint16_t opcode = ((uint16_t *) m328p->flash)[m328p->program_counter];

    if (M328P_INSTR_OUT (opcode))
    { // 'OUT'
        __m328p_instr_out (m328p, opcode);
        m328p->program_counter += sizeof (uint16_t);
    }
    else if (M328P_INSTR_JMP (opcode))
    { // 'JMP'
        const uint16_t extra = ((uint16_t *) m328p->flash)[m328p->program_counter + 1];
        __m328p_instr_jmp (m328p, opcode, extra);
        m328p->program_counter += sizeof (uint32_t);
    }
    else if (M328P_INSTR_IN (opcode))
    { // 'IN'
        __m328p_instr_in (m328p, opcode);
        m328p->program_counter += sizeof (uint16_t);
    }
    else if (M328P_INSTR_ADC (opcode))
    { // 'ADC'
        __m328p_instr_add_adc (m328p, opcode, true);
        m328p->program_counter += sizeof (uint16_t);
    }
    else if (M328P_INSTR_ADD (opcode))
    { // 'ADD'
        __m328p_instr_add_adc (m328p, opcode, false);
        m328p->program_counter += sizeof (uint16_t);
    }
    else if (M328P_INSTR_EOR (opcode))
    { // 'EOR'
        m328p->program_counter += sizeof (uint16_t);
    }
    else
    {
        M328P_RETURN_ERR (M328P_ERR_UNRECOGNIZED_OPCODE);
    }

    return M328P_ERR_OK;
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
void __m328p_instr_out (m328p_t *m328p, uint16_t opcode)
{    const uint32_t A = ((opcode & 0b0000000000001111)
        | ((opcode & 0b0000011000000000) >> 9)); // Destination Address (I/O Space)
    const uint32_t Rr = ((opcode & 0b0000000111110000) >> 4); // Source Register


    const uint8_t Rr_value = __m328p_read_general_register (m328p, Rr);
    __m328p_write_io_register (m328p, A, Rr_value);

    m328p->cycles += M328P_INSTR_OUT_CYCLES;
}

/// Executes the 'IN' instruction.
void __m328p_instr_in (m328p_t *m328p, uint16_t opcode)
{
    const uint32_t A = ((opcode & 0b0000000000001111)
        | ((opcode & 0b0000011000000000) >> 9)); // Destination Address (I/O Space)
    const uint32_t Rd = ((opcode & 0b0000000111110000) >> 4); // Destination Register

    const uint8_t A_value  = __m328p_read_io_register (m328p, A);
    __m328p_write_general_register (m328p, Rd, A_value);

    m328p->cycles += M328P_INSTR_IN_CYCLES;
}

/// Executes the 'JMP' instruction.
void __m328p_instr_jmp (m328p_t *m328p, uint16_t opcode, uint16_t extra)
{
    m328p_pointer k = ((uint32_t) extra)
        | ((opcode & 0b0000000111110000) << 5)
        | ((opcode & 0b0000000000000001) << 8); // Constant Address
    
    k *= 2; // Multiply by two to make the address match instruction.

    __M328P_VERBOSE (printf ("JMP to 0x%.6x\n", k));

    m328p->program_counter = k;
    m328p->cycles += M328P_INSTR_JMP_CYCLES;
}

/// Executes the 'EOR' instruction.
void __m328p_instr_eor ((m328p_t *m328p, uint16_t opcode)
{

}

/// Executes the 'ADD' and 'ADC' instruction.
void __m328p_instr_add_adc (m328p_t *m328p, uint16_t opcode, bool add_carry)
{
    // Reads the arguments.
    const uint8_t Rr = (((opcode & 0b0000001000000000) >> 5)
        | (opcode & 0b0000000000001111));
    const uint8_t Rd = (((opcode & 0b0000000100000000) >> 4)
        | ((opcode & 0b0000000011110000) >> 4));
    
    // Reads the SREG value, and the two numbers to add.
    uint8_t SREG_value = __m328p_read_sreg (m328p);
    const uint8_t Rr_value = __m328p_read_general_register (m328p, Rr), Rd_value = __m328p_read_general_register (m328p, Rd);

    // If we want to do 'ADC' check for the carry flag in the CPU, and possible set it.
    bool carry_flag = add_carry ? (SREG_value & M328P_SREG_C) : 0;
    uint8_t res = 0;

    // Loops over each individual bit, and performs addition with carry flag.
    for (uint8_t i = 0; i < 8; ++i)
    {
        const uint8_t ba = Rd_value & _BV8 (i);
        const uint8_t bb = Rr_value & _BV8 (i);
        if (carry_flag)
        {
            if (ba && bb)
            {
                res |= _BV8 (i);
                carry_flag = true;
            }
            else if (ba || bb)
            {
                carry_flag = true;
            }
            else
            {
                res |= _BV8 (i);
                carry_flag = false;
            }
        }
        else
        {
            if (ba && bb)
            {
                carry_flag = true;
            }
            else if (ba || bb)
            {
                res |= _BV8 (i);
            }
        }

        // Set H if there was a carry_flag from bit 3.
        if (carry_flag && i == 3) SREG_value |= M328P_SREG_H;
        else SREG_value &= ~M328P_SREG_H;
    }

    // Set N if the MSB of the result is set.
    if (res & _BV8 (7)) SREG_value |= M328P_SREG_N;
    else SREG_value &= ~M328P_SREG_N;

    // Set Z if the result is zero.
    if (res == 0) SREG_value |= M328P_SREG_Z;
    else SREG_value &= ~M328P_SREG_Z;

    // Set the C flag if the carry_flag flag remains set after addition.
    if (carry_flag) SREG_value |= M328P_SREG_C;
    else SREG_value &= ~M328P_SREG_C;

    m328p->cycles += M328P_INSTR_ADC_ADD_CYCLES;
}
