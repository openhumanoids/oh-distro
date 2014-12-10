/*
 * Color.cc
 *
 *  Created on: Feb 2, 2012
 *      Author: mfleder
 */

#include "Color.h"
#include "SurrogateException.h"

#include <iostream>
using namespace std;

namespace surrogate_gui
{

	Color::StockColor Color::ColorArray[12] =
		{
				RED,
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


	/**Get a set of colors different from color
	 * @param color the color we do not want in the return set
	* @param numColors number of colors we want in the return set*/
	std::vector<Color::StockColor> Color::getDifferentColors(Color::StockColor color, int numColors)
	{
		if (numColors <= 0)
			throw SurrogateException("getDifferentColors called with non-positive numColors");

		const int highlight_int = HIGHLIGHT;
		//--
		std::vector<Color::StockColor> returnColors;
		returnColors.reserve(numColors);
		Color::StockColor nextStockColor = color;
		for (int i = 0; (int) returnColors.size() < numColors; i++)
		{
			int nextColorAsInt = i % highlight_int;
			nextStockColor = getColor(nextColorAsInt);
			if (nextStockColor == color)
			{
				continue; //skip over the given color
			}
			returnColors.push_back(nextStockColor);
		}
		return returnColors;
	}

	void Color::getColor(Color::StockColor color, RGB_PCL &rgb)
	{
		switch (color)
		{
			case HIGHLIGHT:
				//cout << endl << "Highlight" << endl;
				set(rgb, 128, 255, 0);
				break;
			case RED:
				//cout << endl << "Red" << endl;
				set(rgb, 255, 0, 0);
				break;
			case GREEN:
				//cout << endl << "Green" << endl;
				set(rgb, 0, 255, 0);
				break;
			case BLUE:
				//cout << endl << "Blue" << endl;
				set(rgb, 0, 0, 255);
				break;
			case YELLOW:
				//cout << endl << "Yellow" << endl;
				set(rgb, 255, 255, 0);
				break;
			case PURPLE:
				//cout << endl << "Purple" << endl;
				set(rgb, 233, 0, 255);
				break;
			/*case BLACKBERRY:
				//cout << endl << "Blackberry" << endl;
				set(rgb, 77, 1, 53);
				break;
			case RED_VIOLET:
				//cout << endl << "Red Violet" << endl;
				set(rgb, 219, 112, 147);
				break;*/
			case ORANGE:
				//cout << endl << "Orange" << endl;
				set(rgb, 255, 128, 0);
				break;
			/*case SALMON:
				//cout << endl << "Salmon" << endl;
				set(rgb, 250, 128, 114);
				break;*/
			case LAWN_GREEN:
				//cout << endl << "Lawn Green" << endl;
				set(rgb, 124, 252, 0);
				break;
			default: //NULL_COLOR for example
				throw SurrogateException("getColor(color, numColors): unknown color: " + to_string((int) color));
				//color3f[0] = 0.5;
				//color3f[1] = 1.0;
				//color3f[2] = 0.0;
				break;
		}
			return;
	}


	void Color::set(RGB_PCL &rgb, unsigned char r, unsigned char g, unsigned char b)
	{
		rgb.red  	= r*255;
		rgb.green 	= g*255;
		rgb.blue 	= b*255;
	}

	double Color::toRgbPclFloat(unsigned char r, unsigned char g, unsigned char b)
	{
		RGB_PCL c;
		c.red = r;
		c.green = g;
		c.blue = b;
		return c.float_value;
	}


	/**returns the color_th color except highlight.
	 * will wrap and start dupliating after #(highlight) colors.
	 * ie. highlight_th color = 0*/
	Color::StockColor Color::getColor(int color)
	{
		if (color < 0)
			throw SurrogateException("Invalid color passed to getColor: " + to_string(color));
		color = color % (int) HIGHLIGHT;

		return Color::ColorArray[color];
	}

	void Color::extractColor(const double rgb_pcl_float_value,
							 unsigned char &r, unsigned char &g, unsigned char &b)
	{
		RGB_PCL pclColor;
		pclColor.float_value = rgb_pcl_float_value;
		r = pclColor.red;
		g = pclColor.green;
		b = pclColor.blue;
	}

} //namespace surrogate_gui
