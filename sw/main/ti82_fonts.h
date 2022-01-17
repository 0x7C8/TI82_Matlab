/** @file fonts.h
 * @brief Fonts used for conversion from ASCII to 7x5 TI82 bitmap patterns
 *
 *	This file is mostly a copy of the original 
 *	ETF's Nolleblink implementation
 *
 * @author Georgij
 * @date 17 January 2022
 */ 


#ifndef TI82_FONTS_H_
#define TI82_FONTS_H_

#define X )<<1|1	/**< Used to convert 'X's to '1's */
#define _ )<<1		/**< Used to convert '_'s to '0's *  */
#define S ((((((((0 /**< Initializes column */

/** Font data */
const unsigned char font[219*5] =
{
	// 32
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ X X _ X X X X ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ X X X ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ X X X ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ X _ X _ _ ,
	S _ X X X X X X X ,
	S _ _ _ X _ X _ _ ,
	S _ X X X X X X X ,
	S _ _ _ X _ X _ _ ,

	S _ _ _ _ _ X _ _ ,
	S _ _ X _ X _ X _ ,
	S _ X X X X X X X ,
	S _ _ X _ X _ X _ ,
	S _ _ _ X _ _ _ _ ,

	S _ _ X _ _ _ X X ,
	S _ _ _ X _ _ X X ,
	S _ _ _ _ X _ _ _ ,
	S _ X X _ _ X _ _ ,
	S _ X X _ _ _ X _ ,

	S _ _ X X _ X X _ ,
	S _ X _ _ X _ _ X ,
	S _ X _ X _ X _ X ,
	S _ _ X _ _ _ X _ ,
	S _ X _ X _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ X X ,
	S _ _ _ _ _ X _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ X X X _ _ ,
	S _ _ X _ _ _ X _ ,
	S _ X _ _ _ _ _ X ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ X _ _ _ _ _ X ,
	S _ _ X _ _ _ X _ ,
	S _ _ _ X X X _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ X _ X _ _ ,
	S _ _ _ _ X _ _ _ ,
	S _ _ X X X X X _ ,
	S _ _ _ _ X _ _ _ ,
	S _ _ _ X _ X _ _ ,

	S _ _ _ _ X _ _ _ ,
	S _ _ _ _ X _ _ _ ,
	S _ _ X X X X X _ ,
	S _ _ _ _ X _ _ _ ,
	S _ _ _ _ X _ _ _ ,

	S _ X _ X _ _ _ _ ,
	S _ _ X X _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ X _ _ _ ,
	S _ _ _ _ X _ _ _ ,
	S _ _ _ _ X _ _ _ ,
	S _ _ _ _ X _ _ _ ,
	S _ _ _ _ X _ _ _ ,

	S _ X X _ _ _ _ _ ,
	S _ X X _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ X _ _ _ _ _ ,
	S _ _ _ X _ _ _ _ ,
	S _ _ _ _ X _ _ _ ,
	S _ _ _ _ _ X _ _ ,
	S _ _ _ _ _ _ X _ ,

	// 48
	S _ _ X X X X X _ ,
	S _ X _ X _ _ _ X ,
	S _ X _ _ X _ _ X ,
	S _ X _ _ _ X _ X ,
	S _ _ X X X X X _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ X _ _ _ _ X _ ,
	S _ X X X X X X X ,
	S _ X _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ X _ _ _ _ X _ ,
	S _ X X _ _ _ _ X ,
	S _ X _ X _ _ _ X ,
	S _ X _ _ X _ _ X ,
	S _ X _ _ _ X X _ ,

	S _ _ X _ _ _ _ X ,
	S _ X _ _ _ _ _ X ,
	S _ X _ _ _ X _ X ,
	S _ X _ _ X _ X X ,
	S _ _ X X _ _ _ X ,

	S _ _ _ X X _ _ _ ,
	S _ _ _ X _ X _ _ ,
	S _ _ _ X _ _ X _ ,
	S _ X X X X X X X ,
	S _ _ _ X _ _ _ _ ,

	S _ _ X _ _ X X X ,
	S _ X _ _ _ X _ X ,
	S _ X _ _ _ X _ X ,
	S _ X _ _ _ X _ X ,
	S _ _ X X X _ _ X ,

	S _ _ X X X X _ _ ,
	S _ X _ _ X _ X _ ,
	S _ X _ _ X _ _ X ,
	S _ X _ _ X _ _ X ,
	S _ _ X X _ _ _ _ ,

	S _ _ _ _ _ _ _ X ,
	S _ X X X _ _ _ X ,
	S _ _ _ _ X _ _ X ,
	S _ _ _ _ _ X _ X ,
	S _ _ _ _ _ _ X X ,

	S _ _ X X _ X X _ ,
	S _ X _ _ X _ _ X ,
	S _ X _ _ X _ _ X ,
	S _ X _ _ X _ _ X ,
	S _ _ X X _ X X _ ,

	S _ _ _ _ _ X X _ ,
	S _ X _ _ X _ _ X ,
	S _ X _ _ X _ _ X ,
	S _ _ X _ X _ _ X ,
	S _ _ _ X X X X _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ X X _ X X _ ,
	S _ _ X X _ X X _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ X _ X _ X X _ ,
	S _ _ X X _ X X _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ X _ _ _ ,
	S _ _ _ X _ X _ _ ,
	S _ _ X _ _ _ X _ ,
	S _ X _ _ _ _ _ X ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ X _ X _ _ ,
	S _ _ _ X _ X _ _ ,
	S _ _ _ X _ X _ _ ,
	S _ _ _ X _ X _ _ ,
	S _ _ _ X _ X _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ X _ _ _ _ _ X ,
	S _ _ X _ _ _ X _ ,
	S _ _ _ X _ X _ _ ,
	S _ _ _ _ X _ _ _ ,

	S _ _ _ _ _ _ X _ ,
	S _ _ _ _ _ _ _ X ,
	S _ X _ X _ _ _ X ,
	S _ _ _ _ X _ _ X ,
	S _ _ _ _ _ X X _ ,

	S _ _ X X X X X _ ,
	S _ X _ _ _ _ _ X ,
	S _ X _ _ X _ _ X ,
	S _ X _ X _ X _ X ,
	S _ X _ X X X X _ ,

	// 65
	S _ X X X X X X _ ,
	S _ _ _ _ X _ _ X ,
	S _ _ _ _ X _ _ X ,
	S _ _ _ _ X _ _ X ,
	S _ X X X X X X _ ,
	
	S _ X X X X X X X ,
	S _ X _ _ X _ _ X ,
	S _ X _ _ X _ _ X ,
	S _ X _ _ X _ _ X ,
	S _ _ X X _ X X _ ,
	
	S _ _ X X X X X _ ,
	S _ X _ _ _ _ _ X ,
	S _ X _ _ _ _ _ X ,
	S _ X _ _ _ _ _ X ,
	S _ _ X _ _ _ X _ ,
	
	S _ X X X X X X X ,
	S _ X _ _ _ _ _ X ,
	S _ X _ _ _ _ _ X ,
	S _ X _ _ _ _ _ X ,
	S _ _ X X X X X _ ,
	
	S _ X X X X X X X ,
	S _ X _ _ X _ _ X ,
	S _ X _ _ X _ _ X ,
	S _ X _ _ X _ _ X ,
	S _ X _ _ _ _ _ X ,
	
	S _ X X X X X X X ,
	S _ _ _ _ X _ _ X ,
	S _ _ _ _ X _ _ X ,
	S _ _ _ _ X _ _ X ,
	S _ _ _ _ _ _ _ X ,
	
	S _ _ X X X X X _ ,
	S _ X _ _ _ _ _ X ,
	S _ X _ _ X _ _ X ,
	S _ X _ _ X _ _ X ,
	S _ X X X X _ X _ ,
	
	S _ X X X X X X X ,
	S _ _ _ _ X _ _ _ ,
	S _ _ _ _ X _ _ _ ,
	S _ _ _ _ X _ _ _ ,
	S _ X X X X X X X ,
	
	S _ _ _ _ _ _ _ _ ,
	S _ X _ _ _ _ _ X ,
	S _ X X X X X X X ,
	S _ X _ _ _ _ _ X ,
	S _ _ _ _ _ _ _ _ ,
	
	S _ _ X _ _ _ _ _ ,
	S _ X _ _ _ _ _ _ ,
	S _ X _ _ _ _ _ X ,
	S _ _ X X X X X X ,
	S _ _ _ _ _ _ _ X ,
	
	S _ X X X X X X X ,
	S _ _ _ _ X _ _ _ ,
	S _ _ _ X _ X _ _ ,
	S _ _ X _ _ _ X _ ,
	S _ X _ _ _ _ _ X ,
	
	S _ X X X X X X X ,
	S _ X _ _ _ _ _ _ ,
	S _ X _ _ _ _ _ _ ,
	S _ X _ _ _ _ _ _ ,
	S _ X _ _ _ _ _ _ ,
	
	S _ X X X X X X X ,
	S _ _ _ _ _ _ X _ ,
	S _ _ _ _ X X _ _ ,
	S _ _ _ _ _ _ X _ ,
	S _ X X X X X X X ,
	
	S _ X X X X X X X ,
	S _ _ _ _ _ X _ _ ,
	S _ _ _ _ X _ _ _ ,
	S _ _ _ X _ _ _ _ ,
	S _ X X X X X X X ,
	
	S _ _ X X X X X _ ,
	S _ X _ _ _ _ _ X ,
	S _ X _ _ _ _ _ X ,
	S _ X _ _ _ _ _ X ,
	S _ _ X X X X X _ ,
	
	S _ X X X X X X X ,
	S _ _ _ _ X _ _ X ,
	S _ _ _ _ X _ _ X ,
	S _ _ _ _ X _ _ X ,
	S _ _ _ _ _ X X _ ,
	
	S _ _ X X X X X _ ,
	S _ X _ _ _ _ _ X ,
	S _ X _ X _ _ _ X ,
	S _ _ X _ _ _ _ X ,
	S _ X _ X X X X _ ,
	
	S _ X X X X X X X ,
	S _ _ _ _ X _ _ X ,
	S _ _ _ X X _ _ X ,
	S _ _ X _ X _ _ X ,
	S _ X _ _ _ X X _ ,
	
	S _ X _ _ _ X X _ ,
	S _ X _ _ X _ _ X ,
	S _ X _ _ X _ _ X ,
	S _ X _ _ X _ _ X ,
	S _ _ X X _ _ _ X ,
	
	S _ _ _ _ _ _ _ X ,
	S _ _ _ _ _ _ _ X ,
	S _ X X X X X X X ,
	S _ _ _ _ _ _ _ X ,
	S _ _ _ _ _ _ _ X ,
	
	S _ _ X X X X X X ,
	S _ X _ _ _ _ _ _ ,
	S _ X _ _ _ _ _ _ ,
	S _ X _ _ _ _ _ _ ,
	S _ _ X X X X X X ,
	
	S _ _ _ _ X X X X ,
	S _ _ X X _ _ _ _ ,
	S _ X _ _ _ _ _ _ ,
	S _ _ X X _ _ _ _ ,
	S _ _ _ _ X X X X ,
	
	S _ _ X X X X X X ,
	S _ X _ _ _ _ _ _ ,
	S _ _ X X X _ _ _ ,
	S _ X _ _ _ _ _ _ ,
	S _ _ X X X X X X ,
	
	S _ X X _ _ _ X X ,
	S _ _ _ X _ X _ _ ,
	S _ _ _ _ X _ _ _ ,
	S _ _ _ X _ X _ _ ,
	S _ X X _ _ _ X X ,
	
	S _ _ _ _ _ X X X ,
	S _ _ _ _ X _ _ _ ,
	S _ X X X _ _ _ _ ,
	S _ _ _ _ X _ _ _ ,
	S _ _ _ _ _ X X X ,
	
	S _ X X _ _ _ _ X ,
	S _ X _ X _ _ _ X ,
	S _ X _ _ X _ _ X ,
	S _ X _ _ _ X _ X ,
	S _ X _ _ _ _ X X ,
	
	// 91
	S _ X X X X X X X ,
	S _ X _ _ _ _ _ X ,
	S _ X _ _ _ _ _ X ,
	S _ X _ _ _ _ _ X ,
	S _ X _ _ _ _ _ X ,

	S _ _ _ _ _ _ _ X ,
	S _ _ _ _ _ X X _ ,
	S _ _ _ _ X _ _ _ ,
	S _ _ X X _ _ _ _ ,
	S _ X _ _ _ _ _ _ ,

	S _ X _ _ _ _ _ X ,
	S _ X _ _ _ _ _ X ,
	S _ X _ _ _ _ _ X ,
	S _ X _ _ _ _ _ X ,
	S _ X X X X X X X ,

	S _ _ _ _ _ X _ _ ,
	S _ _ _ _ _ _ X _ ,
	S _ _ _ _ _ _ _ X ,
	S _ _ _ _ _ _ X _ ,
	S _ _ _ _ _ X _ _ ,

	S _ X _ _ _ _ _ _ ,
	S _ X _ _ _ _ _ _ ,
	S _ X _ _ _ _ _ _ ,
	S _ X _ _ _ _ _ _ ,
	S _ X _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ X ,
	S _ _ _ _ _ _ X _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	// 97
	S _ _ X _ _ _ _ _ ,
	S _ X _ X _ X _ _ ,
	S _ X _ X _ X _ _ ,
	S _ X _ X _ X _ _ ,
	S _ X X X X _ _ _ ,

	S _ X X X X X X X ,
	S _ X _ _ X _ _ _ ,
	S _ X _ _ _ X _ _ ,
	S _ X _ _ _ X _ _ ,
	S _ _ X X X _ _ _ ,

	S _ _ X X X _ _ _ ,
	S _ X _ _ _ X _ _ ,
	S _ X _ _ _ X _ _ ,
	S _ X _ _ _ X _ _ ,
	S _ _ X _ _ _ _ _ ,

	S _ _ X X X _ _ _ ,
	S _ X _ _ _ X _ _ ,
	S _ X _ _ _ X _ _ ,
	S _ X _ _ X _ _ _ ,
	S _ X X X X X X X ,

	S _ _ X X X _ _ _ ,
	S _ X _ X _ X _ _ ,
	S _ X _ X _ X _ _ ,
	S _ X _ X _ X _ _ ,
	S _ _ _ X X _ _ _ ,

	S _ _ _ _ X _ _ _ ,
	S _ X X X X X X _ ,
	S _ _ _ _ X _ _ X ,
	S _ _ _ _ _ _ _ X ,
	S _ _ _ _ _ _ X _ ,

	S _ _ _ _ X X _ _ ,
	S _ X _ X _ _ X _ ,
	S _ X _ X _ _ X _ ,
	S _ X _ X _ _ X _ ,
	S _ _ X X X X X _ ,

	S _ X X X X X X X ,
	S _ _ _ _ X _ _ _ ,
	S _ _ _ _ _ X _ _ ,
	S _ _ _ _ _ X _ _ ,
	S _ X X X X _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ X _ _ _ X _ _ ,
	S _ X X X X X _ X ,
	S _ X _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ X _ _ _ _ _ ,
	S _ X _ _ _ _ _ _ ,
	S _ X _ _ _ X _ _ ,
	S _ _ X X X X _ X ,
	S _ _ _ _ _ _ _ _ ,

	S _ X X X X X X X ,
	S _ _ X _ _ _ _ _ ,
	S _ _ _ X _ _ _ _ ,
	S _ _ X _ X _ _ _ ,
	S _ X _ _ _ X _ _ ,


	S _ _ _ _ _ _ _ _ ,
	S _ X _ _ _ _ _ X ,
	S _ X X X X X X X ,
	S _ X _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ X X X X X _ _ ,
	S _ _ _ _ _ X _ _ ,
	S _ _ _ X X _ _ _ ,
	S _ _ _ _ _ X _ _ ,
	S _ X X X X _ _ _ ,

	S _ X X X X X _ _ ,
	S _ _ _ _ X _ _ _ ,
	S _ _ _ _ _ X _ _ ,
	S _ _ _ _ _ X _ _ ,
	S _ X X X X _ _ _ ,

	S _ _ X X X _ _ _ ,
	S _ X _ _ _ X _ _ ,
	S _ X _ _ _ X _ _ ,
	S _ X _ _ _ X _ _ ,
	S _ _ X X X _ _ _ ,

	S _ X X X X X _ _ ,
	S _ _ _ X _ X _ _ ,
	S _ _ _ X _ X _ _ ,
	S _ _ _ X _ X _ _ ,
	S _ _ _ _ X _ _ _ ,

	S _ _ _ _ X _ _ _ ,
	S _ _ _ X _ X _ _ ,
	S _ _ _ X _ X _ _ ,
	S _ _ _ X X _ _ _ ,
	S _ X X X X X _ _ ,

	S _ X X X X X _ _ ,
	S _ _ _ _ X _ _ _ ,
	S _ _ _ _ _ X _ _ ,
	S _ _ _ _ _ X _ _ ,
	S _ _ _ _ X _ _ _ ,

	S _ X _ _ X _ _ _ ,
	S _ X _ X _ X _ _ ,
	S _ X _ X _ X _ _ ,
	S _ X _ X _ X _ _ ,
	S _ _ X _ _ _ _ _ ,

	S _ _ _ _ _ X _ _ ,
	S _ _ X X X X X X ,
	S _ X _ _ _ X _ _ ,
	S _ X _ _ _ _ _ _ ,
	S _ _ X _ _ _ _ _ ,

	S _ _ X X X X _ _ ,
	S _ X _ _ _ _ _ _ ,
	S _ X _ _ _ _ _ _ ,
	S _ _ X _ _ _ _ _ ,
	S _ X X X X X _ _ ,

	S _ _ _ X X X _ _ ,
	S _ _ X _ _ _ _ _ ,
	S _ X _ _ _ _ _ _ ,
	S _ _ X _ _ _ _ _ ,
	S _ _ _ X X X _ _ ,

	S _ _ X X X X _ _ ,
	S _ X _ _ _ _ _ _ ,
	S _ _ X X _ _ _ _ ,
	S _ X _ _ _ _ _ _ ,
	S _ _ X X X X _ _ ,

	S _ X _ _ _ X _ _ ,
	S _ _ X _ X _ _ _ ,
	S _ _ _ X _ _ _ _ ,
	S _ _ X _ X _ _ _ ,
	S _ X _ _ _ X _ _ ,

	S _ _ _ _ X X _ _ ,
	S _ X _ X _ _ _ _ ,
	S _ X _ X _ _ _ _ ,
	S _ X _ X _ _ _ _ ,
	S _ _ X X X X _ _ ,

	S _ X _ _ _ X _ _ ,
	S _ X X _ _ X _ _ ,
	S _ X _ X _ X _ _ ,
	S _ X _ _ X X _ _ ,
	S _ X _ _ _ X _ _ ,

	// 123
	S _ _ _ _ X _ _ _ ,
	S _ _ X X _ X X _ ,
	S _ X _ _ _ _ _ X ,
	S _ X _ _ _ _ _ X ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ X X X X X X X ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ X _ _ _ _ _ X ,
	S _ X _ _ _ _ _ X ,
	S _ _ X X _ X X _ ,
	S _ _ _ _ X _ _ _ ,

	S _ _ _ _ X _ _ _ ,
	S _ _ _ _ _ X _ _ ,
	S _ _ _ _ X _ _ _ ,
	S _ _ _ X _ _ _ _ ,
	S _ _ _ _ X _ _ _ ,

	S _ X X X X _ _ _ ,
	S _ X _ _ _ X _ _ ,
	S _ X _ _ _ _ X _ ,
	S _ X _ _ _ X _ _ ,
	S _ X X X X _ _ _ ,

	// 128 (OF)
	S _ _ _ X X X X _ ,
	S _ _ X _ _ _ _ X ,
	S _ _ X _ _ _ _ X ,
	S _ X X _ _ _ _ X ,
	S _ _ _ X _ _ X _ ,

	S _ _ X X X X _ X ,
	S _ X _ _ _ _ _ _ ,
	S _ X _ _ _ _ _ _ ,
	S _ _ X _ _ _ _ _ ,
	S _ X X X X X _ X ,

	// 130
	S _ _ X X X _ _ _ ,
	S _ X _ X _ X X _ ,
	S _ X _ X _ X _ X ,
	S _ X _ X _ X _ _ ,
	S _ _ _ X X _ _ _ ,

	S _ _ X _ _ _ _ _ ,
	S _ X _ X _ X X _ ,
	S _ X _ X _ X _ X ,
	S _ X _ X _ X X _ ,
	S _ X X X X _ _ _ ,

	S _ _ X _ _ _ _ _ ,
	S _ X _ X _ X _ X ,
	S _ X _ X _ X _ _ ,
	S _ X _ X _ X _ X ,
	S _ X X X X _ _ _ ,

	S _ _ X _ _ _ _ _ ,
	S _ X _ X _ X _ _ ,
	S _ X _ X _ X _ _ ,
	S _ X _ X _ X _ X ,
	S _ X X X X _ X _ ,

	S _ _ X _ _ _ _ _ ,
	S _ X _ X _ X _ _ ,
	S _ X _ X _ X _ X ,
	S _ X _ X _ X _ _ ,
	S _ X X X X _ _ _ ,

	S _ _ _ X X _ _ _ ,
	S _ _ X _ _ X _ _ ,
	S _ _ X _ _ X _ _ ,
	S _ X X _ _ X _ _ ,
	S _ _ X _ _ X _ _ ,

	S _ _ X X X _ _ _ ,
	S _ X _ X _ X X _ ,
	S _ X _ X _ X _ X ,
	S _ X _ X _ X X _ ,
	S _ _ _ X X _ _ _ ,

	S _ _ X X X _ _ _ ,
	S _ X _ X _ X _ X ,
	S _ X _ X _ X _ _ ,
	S _ X _ X _ X _ X ,
	S _ _ _ X X _ _ _ ,

	S _ _ X X X _ _ _ ,
	S _ X _ X _ X _ _ ,
	S _ X _ X _ X _ X ,
	S _ X _ X _ X X _ ,
	S _ _ _ X X _ _ _ ,

	S _ _ _ _ _ _ _ X ,
	S _ X _ _ _ X _ _ ,
	S _ X X X X X _ X ,
	S _ X _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	// 140
	S _ _ _ _ _ _ _ _ ,
	S _ X _ _ X _ X _ ,
	S _ X X X X _ _ X ,
	S _ X _ _ _ _ X _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ X _ _ X _ _ _ ,
	S _ X X X X _ _ X ,
	S _ X _ _ _ _ X _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ X X X X _ _ _ ,
	S _ _ _ X _ X _ X ,
	S _ _ _ X _ X _ _ ,
	S _ _ _ X _ X _ X ,
	S _ X X X X _ _ _ ,

	S _ X X X X _ _ _ ,
	S _ _ _ X _ X _ _ ,
	S _ _ _ X _ X _ X ,
	S _ _ _ X _ X _ _ ,
	S _ X X X X _ _ _ ,

	S _ X X X X X _ _ ,
	S _ X _ X _ X _ _ ,
	S _ X _ X _ X X _ ,
	S _ X _ _ _ X _ X ,
	S _ X _ _ _ X _ _ ,

	S _ _ X _ _ X _ _ ,
	S _ X _ X _ X _ _ ,
	S _ X X X X X _ _ ,
	S _ X _ X _ X _ _ ,
	S _ X _ _ X _ _ _ ,

	S _ X X X X X _ _ ,
	S _ _ _ _ X _ X _ ,
	S _ X X X X X X X ,
	S _ X _ _ X _ _ X ,
	S _ X _ _ X _ _ X ,

	S _ _ X X X _ _ _ ,
	S _ X _ _ _ X X _ ,
	S _ X _ _ _ X _ X ,
	S _ X _ _ _ X X _ ,
	S _ _ X X X _ _ _ ,

	S _ _ X X X _ _ _ ,
	S _ X _ _ _ X _ X ,
	S _ X _ _ _ X _ _ ,
	S _ X _ _ _ X _ X ,
	S _ _ X X X _ _ _ ,

	S _ _ X X X _ _ _ ,
	S _ X _ _ _ X _ _ ,
	S _ X _ _ _ X _ X ,
	S _ X _ _ _ X X _ ,
	S _ _ X X X _ _ _ ,

	// 150
	S _ _ X X X _ _ _ ,
	S _ X _ _ _ _ X _ ,
	S _ X _ _ _ _ _ X ,
	S _ _ X _ _ _ X _ ,
	S _ X X X X _ _ _ ,

	S _ _ X X X _ _ _ ,
	S _ X _ _ _ _ _ _ ,
	S _ X _ _ _ _ _ X ,
	S _ _ X _ _ _ X _ ,
	S _ X X X X _ _ _ ,

	S _ _ _ _ X X _ _ ,
	S _ X _ X _ _ _ X ,
	S _ X _ X _ _ _ _ ,
	S _ X _ X _ _ _ X ,
	S _ _ X X X X _ _ ,

	S _ _ X X X X _ _ ,
	S _ X _ _ _ _ X X ,
	S _ X _ _ _ _ X _ ,
	S _ X _ _ _ _ X X ,
	S _ _ X X X X _ _ ,

	S _ _ X X X X _ _ ,
	S _ X _ _ _ _ _ X ,
	S _ X _ _ _ _ _ _ ,
	S _ X _ _ _ _ _ X ,
	S _ _ X X X X _ _ ,

	S _ X _ X X _ _ _ ,
	S _ _ X _ _ X _ _ ,
	S _ X _ X _ X _ _ ,
	S _ X _ _ X _ _ _ ,
	S _ _ X X _ X _ _ ,

	S _ X X _ X _ _ _ ,
	S _ X _ X X X X _ ,
	S _ X _ _ X _ _ X ,
	S _ X _ _ _ _ _ X ,
	S _ X _ _ _ _ X _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	//160
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	// 170
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	// 180
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	// 190
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ X X X X _ _ _ ,
	S _ _ _ X _ X _ X ,
	S _ _ _ X _ X _ _ ,
	S _ _ _ X _ X _ X ,
	S _ X X X X _ _ _ ,

	S _ X X X X _ _ _ ,
	S _ _ _ X _ X _ _ ,
	S _ _ _ X _ X _ X ,
	S _ _ _ X _ X _ _ ,
	S _ X X X X _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	// 200
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	// 210
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ X X X X _ _ ,
	S _ X _ _ _ _ X X ,
	S _ X _ _ _ _ X _ ,
	S _ X _ _ _ _ X X ,
	S _ _ X X X X _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	// 220
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ X _ _ _ _ _ ,
	S _ X _ X _ X _ X ,
	S _ X _ X _ X _ _ ,
	S _ X _ X _ X _ X ,
	S _ X X X X _ _ _ ,
	
	S _ _ X _ _ _ _ _ ,
	S _ X _ X _ X X _ ,
	S _ X _ X _ X _ X ,
	S _ X _ X _ X X _ ,
	S _ X X X X _ _ _ ,

	// 230
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	// 240
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ X X X _ _ _ ,
	S _ X _ _ _ X _ X ,
	S _ X _ _ _ X _ _ ,
	S _ X _ _ _ X _ X ,
	S _ _ X X X _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,

	// 250
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
	S _ _ _ _ _ _ _ _ ,
};

#endif