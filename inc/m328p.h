#pragma once

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <stdarg.h>

#include "defs.h"

///////////////////////////////////////////////////////////////////////////
// Naming Convention
///////////////////////////////////////////////////////////////////////////

/*
Rd:     Destination (and source) register in the Register File
Rr:     Source register in the Register File
R:      Result after instruction is executed
K:      Constant data
k:      Constant address
b:      Bit in the Register File or I/O Register (3-bit)
s:      Bit in the Status Register (3-bit)
X,Y,Z:  Indirect Address Register (X=R27:R26, Y=R29:R28, and Z=R31:R30)
A:      I/O location address
q:      Displacement for direct addressing (6-bit)
*/

///////////////////////////////////////////////////////////////////////////
// Registers
///////////////////////////////////////////////////////////////////////////

#define M328P_SREG_IO_ADDR      0x3F
#define M328P_SREG_I            _BV8 (7)    // Global Interrupt Enable
#define M328P_SREG_T            _BV8 (6)    // Bit Copy Storage
#define M328P_SREG_H            _BV8 (5)    // Half Carry Flag
#define M328P_SREG_S            _BV8 (4)    // Sign Bit
#define M328P_SREG_V            _BV8 (3)    // Two's Complement Overflow Flag
#define M328P_SREG_N            _BV8 (2)    // Negative Flag
#define M328P_SREG_Z            _BV8 (1)    // Zero Flag
#define M328P_SREG_C            _BV8 (0)    // Carry Flag

#define M328P_SPL_IO_ADDR       0x3D
#define M328P_SPH_IO_ADDR       0x3E

#define M328P_GEN_REGS_START    ((m328p_pointer) 0x0000)
#define M328P_GEN_REGS_END      ((m328p_pointer) 0x001F)
#define M328P_GEN_REG_OFFSET(N) (M328P_GEN_REGS_START + ((m328p_pointer) (N) * (m328p_pointer) sizeof (uint8_t)))

#define M328P_IO_REGS_START     ((m328p_pointer) 0x0020)
#define M328P_IO_REGS_END       ((m328p_pointer) 0x005F)
#define M328P_IO_REG_OFFSET(N)  (M328P_IO_REGS_START + ((m328p_pointer) (N) * (m328p_pointer) sizeof (uint8_t)))

#define M328P_EIO_REGS_START    ((m328p_pointer) 0x0060)
#define M328P_EIO_REGS_END      ((m328p_pointer) 0x00FF)

#define M328P_SRAM_START        ((m328p_pointer) 0x0100)
#define M328P_SRAM_END          ((m328p_pointer) 0x08FF)
#define M328P_SRAM_OFFSET(N)    (N)

#define M328P_GEN_REG_COUNT     ((uint32_t) 32)
#define M328P_GEN_REG_MAX       (M328P_GEN_REG_COUNT - (uint32_t) 1)

#define M328P_IO_REG_COUNT      ((uint32_t) 64)
#define M328P_IO_REG_MAX        (M328P_IO_REG_COUNT - (uint32_t) 1)

///////////////////////////////////////////////////////////////////////////
// Atmega328P Instruction Checkers
///////////////////////////////////////////////////////////////////////////

#define M328P_INSTR_OUT(OP)     ((OP & 0b1111100000000000) == 0b1011100000000000)
#define M328P_INSTR_IN(OP)      ((OP & 0b1111100000000000) == 0b1011000000000000)
#define M328P_INSTR_JMP(OP)     ((OP & 0b1111111000001110) == 0b1001010000001100)
#define M328P_INSTR_ADC(OP)     ((OP & 0b1111110000000000) == 0b0001110000000000)
#define M328P_INSTR_ADD(OP)     ((OP & 0b1111110000000000) == 0b0000110000000000)
#define M328P_INSTR_EOR(OP)     ((OP & 0b1111110000000000) == 0b0010010000000000)
#define M328P_INSTR_LDI(OP)     ((OP & 0b1111000000000000) == 0b1110000000000000)
#define M328P_INSTR_CALL(OP)    ((OP & 0b1111111000001110) == 0b1001010000001110)
#define M328P_INSTR_SBI(OP)     ((OP & 0b1111111100000000) == 0b1001101000000000)
#define M328P_INSTR_SUBI(OP)    ((OP & 0b1111000000000000) == 0b0101000000000000)

///////////////////////////////////////////////////////////////////////////
// Atmega328P Instruction Timing
///////////////////////////////////////////////////////////////////////////

#define M328P_INSTR_OUT_CYCLES      1
#define M328P_INSTR_IN_CYCLES       1
#define M328P_INSTR_JMP_CYCLES      3
#define M328P_INSTR_EOR_CYCLES      3
#define M328P_INSTR_LDI_CYCLES      3
#define M328P_INSTR_ADC_ADD_CYCLES  1
#define M328P_INSTR_CALL_CYCLES     4
#define M328P_INSTR_SBI_CYCLES      2
#define M328P_INSTR_SUBI_CYCLES     1

///////////////////////////////////////////////////////////////////////////
// Atmega328P Device Properties
///////////////////////////////////////////////////////////////////////////

#define M328P_FLASH             _KB (32)
#define M328P_SRAM              _KB (2)
#define M328P_EEPROM            _KB (1)

///////////////////////////////////////////////////////////////////////////
// Atmega328P Types
///////////////////////////////////////////////////////////////////////////

typedef uint16_t m328p_pointer;

#define M328P_GEN_REG_ADDR(N)   ((m328p_pointer) N)
#define M328P_RETURN_ERR(N) \
    fprintf (stderr, "Atmega328P error (" #N ") in file \"%s\" at line %d\n", __FILE__, __LINE__); \
    return N

#define M328P_VERBOSE_PRINTF(M328P, FMT, ...) \
    __M328P_VERBOSE (m328p_printf (M328P, FMT, __VA_ARGS__))
#define M328P_VERBOSE_PRINTF_INSTRUCTION(M328P, INSTRUCTION, FMT, ...) \
    __M328P_VERBOSE (m328p_printf (M328P, "'" INSTRUCTION "':\t" FMT, __VA_ARGS__))

typedef struct
{
    uint8_t         flash[M328P_FLASH];
    uint8_t         sram[M328P_SRAM];
    uint8_t         eeprom[M328P_EEPROM];
    m328p_pointer   program_counter; // No register? Bruh.
    // --- //
    size_t          cycles;
} m328p_t;

typedef enum
{
    M328P_ERR_OK,
    M328P_ERR_NOT_ENOUGH_FLASH,
    M328P_ERR_UNRECOGNIZED_OPCODE
} m328p_err_t;

///////////////////////////////////////////////////////////////////////////
// Atmega328P Subroutines
///////////////////////////////////////////////////////////////////////////

/// Creates an new Atmega328P.
m328p_t *m328p_create (void);

/// Print something with the Atmega328P Prefix.
void m328p_printf (const m328p_t *m328p, const char *fmt, ...);

/// Flashes the given program onto the Atmega328P.
m328p_err_t m328p_flash (m328p_t *m328p, uint8_t *program, size_t program_size);

/// Reboots the Atmega328P.
m328p_err_t m328p_reboot (m328p_t *m328p);

/// Runs the Atmega328P for n cycles.
m328p_err_t m328p_run (m328p_t *m328p, size_t cycles);

/// Executes an single instruction on the Atmega328P.
m328p_err_t m328p_exec_single_instruction (m328p_t *m328p);

/// Frees the given Atmega328P.
void m328p_free (m328p_t *m328p);

/// Writes to an general register in the Atmega328P.
void __m328p_write_general_register (m328p_t *m328p, uint32_t n, uint8_t value);

/// Reads from an general register in the Atmega328P.
uint8_t __m328p_read_general_register (m328p_t *m328p, uint32_t n);

/// Writes to an I/O Register.
void __m328p_write_io_register (m328p_t *m328p, uint32_t a, uint8_t value);

/// Reads from an I/O Register.
uint8_t __m328p_read_io_register (m328p_t *m328p, uint32_t a);

/// Writes to the general SRAM.
void __m328p_write_sram (m328p_t *m328p, m328p_pointer address, uint8_t value);

/// reads from the general SRAM.
uint8_t __m328p_read_sram (m328p_t *m328p, m328p_pointer address);

/// Pushes the given value onto the stack.
void __m328p_push_onto_stack (m328p_t *m328p, uint8_t value);

///////////////////////////////////////////////////////////////////////////
// Atmega328P Inline Subroutines
///////////////////////////////////////////////////////////////////////////

/// Reads the stack pointer.
inline m328p_pointer __m328p_read_spx (m328p_t *m328p)
{
    const uint8_t SPL = __m328p_read_io_register (m328p, M328P_SPL_IO_ADDR);
    const uint8_t SPH = __m328p_read_io_register (m328p, M328P_SPH_IO_ADDR);

    return ((m328p_pointer) SPL | (((m328p_pointer) SPH) << 8));
}

/// Writes the stack pointer.
inline void __m328p_write_spx (m328p_t *m328p, m328p_pointer value)
{
    __m328p_write_io_register (m328p, M328P_SPL_IO_ADDR, (value & 0x0FF));
    __m328p_write_io_register (m328p, M328P_SPH_IO_ADDR, ((value & 0xFF0) >> 8));
}

/// Reads the SREG.
inline uint8_t __m328p_read_sreg (m328p_t *m328p)
{
    return __m328p_read_io_register (m328p, M328P_SREG_IO_ADDR);
}

/// Writes the SREG.
inline void __m328p_write_sreg (m328p_t *m328p, uint8_t value)
{
    __m328p_write_io_register (m328p, M328P_SREG_IO_ADDR, value);
}

///////////////////////////////////////////////////////////////////////////
// Atmega328P Instruction Subroutines
///////////////////////////////////////////////////////////////////////////

/// Executes the 'OUT' instruction.
void __m328p_instr_out (m328p_t *m328p, uint16_t opcode);

/// Executes the 'IN' instruction.
void __m328p_instr_in (m328p_t *m328p, uint16_t opcode);

/// Executes the 'JMP' instruction.
void __m328p_instr_jmp (m328p_t *m328p, uint16_t opcode, uint16_t extra);

/// Executes the 'EOR' instruction.
void __m328p_instr_eor (m328p_t *m328p, uint16_t opcode);

/// Executes the 'ADC' and 'ADD' instruction.
void __m328p_instr_add_adc (m328p_t *m328p, uint16_t opcode, bool add_cary);

/// Executes the 'LDI' instruction.
void __m328p_instr_ldi (m328p_t *m328p, uint16_t opcode);

/// Executes the 'CALL' instruction.
void __m328p_instr_call (m328p_t *m328p, uint16_t opcode, uint16_t extra);

/// Executes the 'SBI' instruction.
void __m328p_instr_sbi (m328p_t *m328p, uint16_t opcode);

/// Executes the 'SUBI' instruction.
void __m328p_instr_subi (m328p_t *m328p, uint16_t opcode);
