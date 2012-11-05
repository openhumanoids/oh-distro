/*
 * AffordancesModelColumns.h
 *
 *  Created on: Oct 26, 2012
 *      Author: drc
 */

#ifndef AFFORDANCESMODELCOLUMNS_H_
#define AFFORDANCESMODELCOLUMNS_H_

using namespace Gtk;

class AffordancesModelColumns : public TreeModelColumnRecord {
public:
	AffordancesModelColumns();
	virtual ~AffordancesModelColumns();

	TreeModelColumn<Glib::ustring> m_col_text;
	TreeModelColumn<int> m_col_number;
};

#endif /* AFFORDANCESMODELCOLUMNS_H_ */
