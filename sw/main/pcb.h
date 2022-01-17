/** @file pcb.h
 * @brief Header file for pin macros.
 *
 * @author Georgij
 * @date 17 January 2022
 */ 

#ifndef PCB_H_
#define PCB_H_

// Expander SPI
#define EXPANDER_CS     26 // CONFIG_EXPANDER_CS
#define EXPANDER_SCK    27 // CONFIG_EXPANDER_SCK
#define EXPANDER_DATA   14 // CONFIG_EXPANDER_DATA
// Keyboard columns
#define KEYBOARD_COL0   36 // CONFIG_KEYBOARD_COL0
#define KEYBOARD_COL1   13 // CONFIG_KEYBOARD_COL1
#define KEYBOARD_COL2   34 // CONFIG_KEYBOARD_COL2 NOTE: PATCHED to GPIO34
#define KEYBOARD_COL3   22 // CONFIG_KEYBOARD_COL3
#define KEYBOARD_COL4   23 // CONFIG_KEYBOARD_COL4
#define KEYBOARD_ROW0   33 // CONFIG_KEYBOARD_ROW0
#define KEYBOARD_ROW1   25 // CONFIG_KEYBOARD_ROW1
#define KEYBIARD_ON     32
// Keyboard column pin mask
#define KEYBOARD_COLS  ((1ULL<<KEYBOARD_COL0) | (1ULL<<KEYBOARD_COL1) | (1ULL<<KEYBOARD_COL2) | (1ULL<<KEYBOARD_COL3) | (1ULL<<KEYBOARD_COL4))

// Display IO
#define DISPLAY_RST     21 // Reset
#define DISPLAY_CE      19 // Chip Enable
#define DISPLAY_WR      18 // Write
#define DISPLAY_DI      17 // Data/Instruction
#define DISPLAY_STB     16 // Standby
// Display IO pin mask
#define DISPLAY_PINS  ((1ULL<<DISPLAY_RST) | (1ULL<<DISPLAY_CE) | (1ULL<<DISPLAY_WR) | (1ULL<<DISPLAY_DI) | (1ULL<<DISPLAY_STB))

#define BAT_ADC         35

#endif /* PCB_H_ */