/**
 * @file References.h
 * @brief iSAM references in doxygen format.
 * @author Michael Kaess
 * @version $Id: Tutorial.h 2898 2010-08-24 01:06:18Z kaess $
 *
 * Copyright (C) 2009-2010 Massachusetts Institute of Technology.
 * Michael Kaess, Hordur Johannsson and John J. Leonard
 *
 * This file is part of iSAM.
 *
 * iSAM is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the
 * Free Software Foundation; either version 2.1 of the License, or (at
 * your option) any later version.
 *
 * iSAM is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 * License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with iSAM.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/** @page Bibliography Bibliography

A list of iSAM-related publications in BibTeX format.

@section bib_journal Journal Papers

@verbatim

% most relevant publication for covariance recovery
@Article{Kaess09ras,
  author =       {M. Kaess and F. Dellaert},
  fullauthor =   {Michael Kaess and Frank Dellaert},
  title =        {Covariance Recovery from a Square Root Information
                  Matrix for Data Association},
  doi =          {10.1016/j.robot.2009.06.008},
  journal =      {Journal of Robotics and Autonomous Systems (RAS)},
  volume =       57,
  issue =        12,
  month =        {Dec},
  pages =        {1198-1210},
  year =         2009,
}

% most relevant publication for iSAM
@Article{Kaess08tro,
  author =       {M. Kaess and A. Ranganathan and F. Dellaert},
  fullauthor =   {Michael Kaess and Ananth Ranganathan and Frank Dellaert},
  title =        {{iSAM}: Incremental Smoothing and Mapping},
  journal =      {IEEE Trans. on Robotics (TRO)},
  volume =       {24},
  number =       {6},
  pages =        {1365-1378},
  month =        {Dec},
  year =         2008,
}

% most relevant publication for Square Root SAM
@Article{Dellaert06ijrr,
  author =       {F. Dellaert and M. Kaess},
  fullauthor =   {Frank Dellaert and Michael Kaess},
  title =        {Square {Root} {SAM}: Simultaneous Localization and Mapping
                  via Square Root Information Smoothing},
  journal =      {Intl. J. of Robotics Research (IJRR)},
  volume =       25,
  number =       12,
  pages =        {1181-1204},
  month =        {Dec},
  year =         2006,
}

@endverbatim

@section bib_other Other Publications

@verbatim

% Bayes Tree
@TechReport{Kaess10tr,
  author =       {M. Kaess and V. Ila and R. Roberts and F. Dellaert},
  fullauthor =   {Michael Kaess and Viorela Ila and Richard Roberts
                  and Frank Dellaert},
  title =        {The {B}ayes Tree: Enabling Incremental Reordering
                  and Fluid Relinearization for Online Mapping},
  institution =  {Computer Science and Artificial Intelligence
                  Laboratory, MIT},
  month =        {Jan},
  year =         2010,
  number =       {MIT-CSAIL-TR-2010-021},
}

% anchor nodes
@InProceedings{Kim10icra,
  author =       {B. Kim and M. Kaess and L. Fletcher and J. Leonard
                  and A. Bachrach and N. Roy and S. Teller},
  fullauthor =   {Been Kim and Michael Kaess and Luke Fletcher and
                  John Leonard and Abe Bachrach and Nicholas Roy and
                  Seth Teller},
  title =        {Multiple Relative Pose Graphs for Robust Cooperative
                  Mapping},
  booktitle =    {IEEE Intl. Conf. on Robotics and Automation (ICRA)},
  address =      {Anchorage, Alaska},
  pages =        {3185-3192},
  month =        {May},
  year =         2010,
}

% iSAM + covariances, for journal paper see Kaess08tro
@InProceedings{Kaess07icra,
  author =       {M. Kaess and A. Ranganathan and F. Dellaert},
  fullauthor =   {Michael Kaess and Ananth Ranganathan and Frank
                  Dellaert},
  title =        {{iSAM}: Fast Incremental Smoothing and Mapping with
                  Efficient Data Association},
  booktitle =    {IEEE Intl. Conf. on Robotics and Automation (ICRA)},
  address =      {Rome, Italy},
  pages =        {1670-1677},
  month =        {Apr},
  year =         2007,
}

% first iSAM paper, for journal paper see Kaess08tro
@InProceedings{Kaess07ijcai,
  author =       {M. Kaess and A. Ranganathan and F. Dellaert},
  fullauthor =   {Michael Kaess and Ananth Ranganathan and Frank
                  Dellaert},
  title =        {Fast Incremental Square Root Information Smoothing},
  booktitle =    {Intl. Joint Conf. on Artificial Intelligence (IJCAI)},
  address =      {Hyderabad, India},
  pages =        {2129-2134},
  month =        {Jan},
  year =         2007,
}

% first Square Root SAM paper, for journal paper see Dellaert06ijrr
@InProceedings{Dellaert05rss,
  author =      {F. Dellaert},
  title =       {Square {Root} {SAM}: Simultaneous Location and
                 Mapping via Square Root Information Smoothing},
  booktitle =   {Robotics: Science and Systems (RSS)},
  year =        2005,
}

@endverbatim

*/
