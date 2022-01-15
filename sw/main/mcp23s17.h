/** @file mcp23s17.h
 * @brief Header file for MCP23S17 macros.
 *
 * For more information about the registers,
 * read MCP23S17's datasheet.
 *
 * @author Georgij
 * @date 11 January 2022
 */ 

#ifndef MCP23S17_H_
#define MCP23S17_H_

// MCP23S17 REGISTERS
/** Data direction register for PORTA. */
#define IODIRA  0x00
/** Data direction register for PORTB. */
#define IODIRB  0x01
/** Configuration register. */
#define IOCONA  0x0A

/** GPIO pull-up resistor register for PORTA. */
#define GPPUA   0x0C
/** GPIO pull-up resistor register for PORTB. */
#define GPPUB   0x0D
/** General purpose I/O PORTA register. */
#define GPIOA   0x12
/** General purpose I/O PORTB register. */
#define GPIOB   0x13

/** Opcode used for communication through SPI */
#define MCP_OPCODE			0x40
/** Value of the last bit used for writing to MCP23S17 through SPI */
#define MCP_WRITE			0x00
/** Value of the last bit used for reading from MCP23S17 through SPI */
#define MCP_READ			0x01

/** The location of HAEN (Hardware Address Enable) bit
*	in MCP23S17's IOCON register. */
#define HAEN	3
/** The location of SEQOP (Sequential Operation) bit
*	in MCP23S17's IOCON register. */
#define SEQOP	5

#endif /* MCP23S17_H_ */