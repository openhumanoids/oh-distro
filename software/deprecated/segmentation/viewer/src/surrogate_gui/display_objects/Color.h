/*
 * Color.h
 *
 *  Created on: Feb 2, 2012
 *      Author: mfleder
 */

#ifndef COLOR_H_
#define COLOR_H_

#include <vector>
#include <string>
#include <sstream>

namespace surrogate_gui
{

	//============pcl rgb color
	typedef union
	{
	  struct /*anonymous*/
	  {
		unsigned char blue;
		unsigned char green;
		unsigned char red;
		unsigned char alpha;
	  };
	  float float_value;
	  long long_value;
	} RGB_PCL;


	//=========Color

	/**Non-instantiable class*/
	class Color
	{
		//fields
		public:

			/**Make sure to update ColorArray if modifying this*/
			enum StockColor
			{
				RED = 0,
				GREEN,
				BLUE,
				YELLOW,
				PURPLE,
				//BLACKBERRY,
				//RED_VIOLET,
				ORANGE,
				//SALMON,
				LAWN_GREEN,
				HIGHLIGHT,
				NULL_COLOR
			};

		static StockColor ColorArray[12];

		//------------methods-------
		static std::vector<Color::StockColor> getDifferentColors(Color::StockColor color, int numColors);
		static void getColor(Color::StockColor color, RGB_PCL &rgb);
		static Color::StockColor getColor(int color);
		static void extractColor(const double rgb_pcl_float_value,
								 unsigned char &r, unsigned char &g, unsigned char &b);
		static double toRgbPclFloat(unsigned char r, unsigned char g, unsigned char b);

		template <class T>
		static std::string to_string(const T &t)
		{
			std::stringstream ss;
			ss << t;
			return ss.str();
		}


		private:
			static void set(RGB_PCL &rgb, unsigned char r, unsigned char g, unsigned char b);

		//non-instantiable
		private:
			Color();
	};
} //namespace surrogate_gui

#endif /* COLOR_H_ */
