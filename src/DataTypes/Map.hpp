/*
 * Map.hpp
 *
 *  Created on: May 5, 2016
 *      Author: shane
 */

#ifndef SRC_DATATYPES_MAP_HPP_
#define SRC_DATATYPES_MAP_HPP_

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <vector>
#include <stdlib.h>

#include <gtsam/base/types.h>
#include <gtsam/geometry/Point3.h>
#include "FileParsing/ParseOptimizationResults.h"

class Map{
private:
	void LoadISCMap(std::vector<std::string>& dates);
	void LoadStandardMap(std::vector<std::string>& dates);
public:
	std::vector<gtsam::Point3> map;
	std::vector<int> landmark_ids;
	std::vector<double> variances;
	std::vector<int> survey_labels;
	int num_surveys;
	std::vector<std::string> _dates;

	std::string _map_base;

    Map(): num_surveys(0) {}

    Map(std::string map_base): num_surveys(0) {
		_map_base = map_base;
	}

	void LoadMap(std::vector<std::string>& dates, bool standard);
    void LoadMap(std::string date);
    void LoadMap(std::string date, const ParseOptimizationResults& pm);

	bool CheckSize() const ;

	int NumSurveys() const ;
};


#endif /* SRC_DATATYPES_MAP_HPP_ */
