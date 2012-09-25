/*
 * ObjectPoints.cpp
 *
 *  Created on: Feb 2, 2012
 *      Author: mfleder
 */

#include "ObjectPoints.h"
#include "SurrogateException.h"

namespace surrogate_gui
{

	//==========constructor/destructor
	ObjectPoints::ObjectPoints(Color::StockColor objColor)
		: color(objColor), indices(SetIntPtr(new std::set<int>)),
		  displaySelection(0)
	{
	}

	ObjectPoints::~ObjectPoints()
	{}

	void ObjectPoints::checkRepInvariant()
	{
		if (displaySelection < 0 || displaySelection > (int) auto_segment_indices.size())
			throw SurrogateException("ObjectPoints displaySelection invalid");
	}


	//============observers
	bool ObjectPoints::displayingAutoSegment() const
	{
		return  indices->size() != 0
				&& auto_segment_indices.size() != 0
				&& displaySelection < (int) auto_segment_indices.size();
	}

	SetIntPtr ObjectPoints::getCurrAutoSegmentDisplay()
	{
		if (!displayingAutoSegment())
			throw SurrogateException("should have called displayingAutoSegment() first.");
		return auto_segment_indices[displaySelection];
	}

	SetIntPtr ObjectPoints::getSegmentBeingDisplayed()
	{
		return   displayingAutoSegment()
					? getCurrAutoSegmentDisplay()
					: indices;
	}

	int ObjectPoints::getDisplaySelection() const
	{
		return displaySelection;
	}

	bool ObjectPoints::displayingCompletePointSet() const
	{
		return (uint) displaySelection == auto_segment_indices.size();
	}

	Color::StockColor ObjectPoints::getColorForAutoSegment(int whichSegment) const
	{
		if (whichSegment < 0 || whichSegment >= (int) auto_segment_indices.size())
			throw SurrogateException("Invalid segment to get color for");

		return (Color::getColor(whichSegment) != color)
			    ? Color::getColor(whichSegment)
				: Color::getColor((whichSegment + 1) % Color::HIGHLIGHT);
	}

	Color::StockColor ObjectPoints::getColorBeingDisplayed() const
	{
		return displayingAutoSegment()
				? getColorForAutoSegment(getDisplaySelection())
				: color;
	}

	//=========================mutators
	/**Deletes the currently displayed segment.
	 * @throws exception if no indices*/
	void ObjectPoints::deleteCurrentSegment()
	{
		if (indices->size() == 0)
			throw SurrogateException("deleteCurrentSegment: No indices to delete");

		if (displayingCompletePointSet())
		{
			clearAutoSegments(); //delete everything
			indices->clear();    // ""
		}
		else if (displayingAutoSegment())
		{
			//erase the currently displayed auto-segment from the big index set
			for (set<int>::iterator indexIter = getCurrAutoSegmentDisplay()->begin();
				indexIter != getCurrAutoSegmentDisplay()->end();
				indexIter++)
			{
				indices->erase(*indexIter);
			}

			//now remove from the auto_segment_indices list
			auto_segment_indices.erase(auto_segment_indices.begin() + displaySelection);

			//display everything left
			setDisplayFullPointSet();
		}
		else
			throw SurrogateException("Should be displaying complete point set or auto segment");
	}

	void ObjectPoints::setDisplayNextSubSegment()
	{
		displaySelection = (displaySelection + 1) % (auto_segment_indices.size() + 1);
		//std::cout << std::endl << "display selection = " << displaySelection << std::endl;
		checkRepInvariant();
	}

	void ObjectPoints::setDisplayPreviousSubSegment()
	{
		displaySelection--;
		if (displaySelection < 0)
			displaySelection = auto_segment_indices.size();
		//std::cout << std::endl << "display selection = " << displaySelection << std::endl;
		checkRepInvariant();
	}

	void ObjectPoints::setDisplayFullPointSet()
	{
		displaySelection = auto_segment_indices.size();
		checkRepInvariant();
	}

	void ObjectPoints::clearAutoSegments()
	{
		auto_segment_indices.clear();
		displaySelection = 0;
	}



} //namespace surrogate_gui


