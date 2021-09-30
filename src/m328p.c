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
    __M328P_VERBOSE (printf ("Executing PC: 0x%.6X, OP: 0x%.4X\n", m328p->program_counter * 2, opcode));

    if (M328P_INSTR_OUT (opcode))
    { // 'OUT'
        __m328p_instr_out (m328p, opcode);
    }
    else if (M328P_INSTR_JMP (opcode))
    { // 'JMP'
        const uint16_t extra = ((uint16_t *) m328p->flash)[m328p->program_counter + 1];
        __m328p_instr_jmp (m328p, opcode, extra);
    }
    else if (M328P_INSTR_IN (opcode))
    { // 'IN'
        __m328p_instr_in (m328p, opcode);
    }
    else if (M328P_INSTR_ADC (opcode))
    { // 'ADC'
        __m328p_instr_add_adc (m328p, opcode, true);
    }
    else if (M328P_INSTR_ADD (opcode))
    { // 'ADD'
        __m328p_instr_add_adc (m328p, opcode, false);
    }
    else if (M328P_INSTR_EOR (opcode))
    { // 'EOR'
        __m328p_instr_eor (m328p, opcode);
    }
    else if (M328P_INSTR_LDI (opcode))
    { // 'LDI'
        __m328p_instr_ldi (m328p, opcode);
    }
    else if (M328P_INSTR_CALL (opcode))
    {
        const uint16_t extra = ((uint16_t *) m328p->flash)[m328p->program_counter + 1];
        __m328p_instr_call (m328p, opcode, extra);
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

/// Writes to the general SRAM.
void __m328p_write_sram (m328p_t *m328p, m328p_pointer address, uint8_t value)
{
    assert ((address >= M328P_SRAM_START) && (address <= M328P_SRAM_END));
    m328p->sram[M328P_SRAM_OFFSET (address)] = value;
}

/// reads from the general SRAM.
uint8_t __m328p_read_sram (m328p_t *m328p, m328p_pointer address)
{
    assert ((address >= M328P_SRAM_START) && (address <= M328P_SRAM_END));
    return m328p->sram[M328P_SRAM_OFFSET (address)];
}

/// Pushes the given value onto the stack.
void __m328p_push_onto_stack (m328p_t *m328p, uint8_t value)
{
    // Reads the stack pointer.
    m328p_pointer SPX_value = __m328p_read_spx (m328p);

    // Pushes the value onto the stack.
    __m328p_write_sram (m328p, SPX_value, value);

    // Increments the stack pointer and writes it.
    ++SPX_value;
    __m328p_write_spx (m328p, SPX_value);
}

///////////////////////////////////////////////////////////////////////////
// Atmega328P Instruction Subroutines
///////////////////////////////////////////////////////////////////////////

/// Executes the 'OUT' instruction.
void __m328p_instr_out (m328p_t *m328p, uint16_t opcode)
{    const uint32_t A = ((opcode & 0b0000000000001111)
        | ((opcode & 0b0000011000000000) >> 5)); // Destination Address (I/O Space)
    const uint32_t Rr = ((opcode & 0b0000000111110000) >> 4); // Source Register

    __M328P_VERBOSE (printf ("OUT from R%u to 0x%.02X\n", Rr, A));

    const uint8_t Rr_value = __m328p_read_general_register (m328p, Rr);
    __m328p_write_io_register (m328p, A, Rr_value);
    
    ++m328p->program_counter;
    m328p->cycles += M328P_INSTR_OUT_CYCLES;
}

/// Executes the 'IN' instruction.
void __m328p_instr_in (m328p_t *m328p, uint16_t opcode)
{
    const uint32_t A = ((opcode & 0b0000000000001111)
        | ((opcode & 0b0000011000000000) >> 9)); // Destination Address (I/O Space)
    const uint32_t Rd = ((opcode & 0b0000000111110000) >> 4); // Destination Register

    __M328P_VERBOSE (printf ("IN from 0x%.02X to R%u\n", A, Rd));

    const uint8_t A_value  = __m328p_read_io_register (m328p, A);
    __m328p_write_general_register (m328p, Rd, A_value);

    ++m328p->program_counter;
    m328p->cycles += M328P_INSTR_IN_CYCLES;
}

/// Executes the 'JMP' instruction.
void __m328p_instr_jmp (m328p_t *m328p, uint16_t opcode, uint16_t extra)
{
    m328p_pointer k = ((uint32_t) extra)
        | ((opcode & 0b0000000111110000) << 5)
        | ((opcode & 0b0000000000000001) << 8); // Constant Address
    
    __M328P_VERBOSE (printf ("JMP to 0x%.6X\n", k * 2));

    m328p->program_counter = k;
    m328p->cycles += M328P_INSTR_JMP_CYCLES;
}

/// Executes the 'EOR' instruction.
void __m328p_instr_eor (m328p_t *m328p, uint16_t opcode)
{
    // Reads the arguments.
    const uint8_t Rr = (((opcode & 0b0000001000000000) >> 5)
        | (opcode & 0b0000000000001111));
    const uint8_t Rd = (((opcode & 0b0000000100000000) >> 4)
        | ((opcode & 0b0000000011110000) >> 4));

    // Reads the SREG value, and the two numbers to XOR.
    uint8_t SREG_value = __m328p_read_sreg (m328p);
    const uint8_t Rr_value = __m328p_read_general_register (m328p, Rr),
        Rd_value = __m328p_read_general_register (m328p, Rd);

    __M328P_VERBOSE (printf ("EOR R%u and R%u\n", Rr, Rd));

    // Performs the XOR operation.
    const uint8_t res = Rr_value ^ Rd_value;

    // Clear V flag.
    SREG_value &= ~M328P_SREG_V;

    // Set N if MSB of the result is set.
    if (res & _BV8 (7)) SREG_value |= M328P_SREG_N;
    else SREG_value &= ~M328P_SREG_N;

    // Set Z if the result is 0x00.
    if (res == 0x00) SREG_value |= M328P_SREG_Z;
    SREG_value &= ~M328P_SREG_Z;

    // Writes to the destination register.
    __m328p_write_general_register (m328p, Rd, res);

    // Writes the new SREG value.
    __m328p_write_sreg (m328p, SREG_value);

    // Adds the clock cycles.
    ++m328p->program_counter;
    m328p->cycles += M328P_INSTR_EOR_CYCLES;
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
    const uint8_t Rr_value = __m328p_read_general_register (m328p, Rr), 
        Rd_value = __m328p_read_general_register (m328p, Rd);

    __M328P_VERBOSE (printf ("%s R%u and R%u\n", add_carry ? "ADC" : "ADD", Rr, Rd));

    // If we want to do 'ADC' check for the carry flag in the CPU, and possible set it.
    bool carry_flag = add_carry ? (SREG_value & M328P_SREG_C) : 0;
    uint8_t res = 0;

    // TODO: Handle negative numbers ?

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

    // Writes the new SREG value.
    __m328p_write_sreg (m328p, SREG_value);

    // Adds the clock cycles.
    ++m328p->program_counter;
    m328p->cycles += M328P_INSTR_ADC_ADD_CYCLES;
}

/// Executes the 'LDI' instruction.
void __m328p_instr_ldi (m328p_t *m328p, uint16_t opcode)
{
    const uint8_t K = (((opcode & 0b0000111100000000) >> 4) | (opcode & 0b0000000000001111));
    const uint8_t Rd = ((opcode & 0b0000000011110000) >> 4) + 16;

    __M328P_VERBOSE (printf ("LDI load value 0x%.02X into R%u\n", K, Rd));

    // Loads the value into the register.
    __m328p_write_general_register (m328p, Rd, K);

    // Adds the clock cycles.
    ++m328p->program_counter;
    m328p->cycles += M328P_INSTR_LDI_CYCLES;
}

/// Executes the 'CALL' instruction.
void __m328p_instr_call (m328p_t *m328p, uint16_t opcode, uint16_t extra)
{
    // Gets the constant address.
    const m328p_pointer k = ((uint32_t) extra)
        | ((opcode & 0b0000000111110000) << 5)
        | ((opcode & 0b0000000000000001) << 8);

    // Prints the verbose stuff.
    __M328P_VERBOSE (printf ("CALL address 0x%.6X\n", k * 2));
    
    // Pushes the current program counter onto the stack.
    __m328p_push_onto_stack (m328p, m328p->program_counter);

    // Sets the program counter value to K.
    m328p->program_counter = k;

    // Adds the clock cycles.
    m328p->cycles += M328P_INSTR_CALL_CYCLES;
}
