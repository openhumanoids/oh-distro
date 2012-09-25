/*
 * ObjectPoints.h
 *
 *  Created on: Feb 2, 2012
 *      Author: mfleder
 */

#ifndef OBJECTPOINTS_H_
#define OBJECTPOINTS_H_

#include "Color.h"

#include <set>
#include <boost/shared_ptr.hpp>

namespace surrogate_gui
{

	typedef boost::shared_ptr<std::set<int> > SetIntPtr;

	/**point cloud of an object in a larger cloud.
	 * Useful class for keeping track of segmentation being performed.*/
	class ObjectPoints
	{
		//--------fields------
		public:
			/**color for this object: the color for anything not an auto-segment.*/
			Color::StockColor color;

			/**point indices in the larger cloud*/
			SetIntPtr indices;

			/**suggested possible segments*/
			std::vector<SetIntPtr > auto_segment_indices;

		private:
			/**which auto-suggested segmented we are displaying.
			 * any of {0, ..., auto_segment_indices.size()}
			 *
			 * If displaySelection == auto_segment_indices.size(), then we
			 * either display the whole object w/o any auto-suggestions
			*/
			int displaySelection;


		//----------constructor/destructor
		public:
			ObjectPoints(Color::StockColor objColor);

			virtual ~ObjectPoints();

			/**make sure state of this object is ok*/
			void checkRepInvariant();

		//------useful observers
		public:
			/**@return true if: (a) indices.size() != 0 &&
			 * 					(b) auto_segmented_indices.size() != 0 &&
			 * 					(c) displaySelection is set to one of the auto-segment sub-components*/
			bool displayingAutoSegment() const;

			/**@throws exception if !displayingAutoSegment*/
			SetIntPtr getCurrAutoSegmentDisplay();

			/**@return whatever set of indices are currently being displayed*/
			SetIntPtr getSegmentBeingDisplayed();

			/**@return which segment is displayed:
			 * elemtn of {0, ..., auto_segment_indices.size()}.
			 * auto_segment_indices.size() indicates we should display the whole object, unpartitioned.*/
			int getDisplaySelection() const;

			/**@return true if should display all of the points as 1 segment*/
			bool displayingCompletePointSet() const;

			/**@param whichSegment: any of {0, ..., auto_segment_indices.size() - 1 }*/
			Color::StockColor getColorForAutoSegment(int whichSegment) const;

			/**return which color is being displayed.
			 * returns this.color if all segments are being displayed*/
			Color::StockColor getColorBeingDisplayed() const;

		//------------mutators
		public:

			/**delete the segment currently being displayed*/
			void deleteCurrentSegment();

			/**display next auto-suggested sub-component*/
			void setDisplayNextSubSegment();

			/**display previous auto-suggested sub-component*/
			void setDisplayPreviousSubSegment();

			void setDisplayFullPointSet();

			/**clear auto_segment_indices and sets displaySelection to 0*/;
			void clearAutoSegments();
	};

	typedef boost::shared_ptr<ObjectPoints> ObjectPointsPtr;

} //namespace surrogate_gui

#endif /* OBJECTPOINTS_H_ */
