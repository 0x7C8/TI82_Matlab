/** @file keymap.h
 * @brief Header file for Keymap macros.
 *
 * @author Georgij
 * @date 24 January 2022
 */ 

#ifndef KEYMAP_H_
#define KEYMAP_H_

// Normal
const char keymap1[10][5] = {
    {0x01, 0x02, 0x03, 0x04, 0x05},
    {0x06, 0x07, 0x08, 0x09, 0x0a},
    {0x0b, 0x0c, 0x0d, 0x0e, 0x0f},
    {0x10, 0x11, 0x12, 0x13, 0x14},
    {0x15, 0x16, 0x17, 0x18, '^'},
    {' ', ',', '(', ')', '/'},
    {' ', '7', '8', '9', '*'},
    {' ', '4', '5', '6', '-'},
    {' ', '1', '2', '3', '+'},
    {' ', '0', '.', '-', 0x00}
};
// TODO : 2nd
const unsigned char keymap2[10][5] = {
    {0x01, 0x02, 0x03, 0x04, 0x05},
    {0x06, 0x07, 0x08, 0x09, 0x0a},
    {0x0b, 0x0c, 0x0d, 0x0e, 0x0f},
    {0x10, 0x11, 0x12, 0x13, 0x14},
    {0x15, 0x16, 0x17, 0x18, '^'},
    {' ', ',', '(', ')', '/'},
    {' ', '7', '8', '9', '*'},
    {' ', '4', '5', '6', '-'},
    {' ', '1', '2', '3', '+'},
    {' ', '0', '.', '-', 0x00}
};
// Alpha
const unsigned char keymapa[10][5] = {
    {0x01, 0x02, 0x03, 0x04, 0x05},
    {0x06, 0x07, 0x08, 0x09, 0x0a},
    {0x0b, 0x0c, 0x0d, 0x0e, 0x0f},
    {'A', 'B', 'C', 0x13, 0x14},
    {'D', 'E', 'F', 'G', 'H'},
    {'I', 'J', 'K', 'L', 'M'},
    {'N', 'O', 'P', 'Q', 'R'},
    {'S', 'T', 'U', 'V', 'W'},
    {'X', 'Y', 'Z', ' ', '"'},
    {' ', ' ', ':', '?', ' '}
};
// TODO: Smol
const unsigned char keymapsmol[10][5] = {
    {0x01, 0x02, 0x03, 0x04, 0x05},
    {0x06, 0x07, 0x08, 0x09, 0x0a},
    {0x0b, 0x0c, 0x0d, 0x0e, 0x0f},
    {'a', 'b', 'c', 0x13, 0x14},
    {'d', 'e', 'f', 'g', 'h'},
    {'i', 'j', 'k', 'l', 'm'},
    {'n', 'o', 'p', 'q', 'r'},
    {'s', 't', 'u', 'v', 'w'},
    {'x', 'y', 'z', ' ', '"'},
    {' ', ' ', ':', '?', ' '}
};

#endif /* KEYMAP_H_ */