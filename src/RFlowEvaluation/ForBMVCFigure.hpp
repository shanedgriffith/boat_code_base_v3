/*
 * ForBMVCFigure.hpp
 *
 *  Created on: May 11, 2016
 *      Author: shane
 */

#ifndef SRC_IMAGEALIGNMENT_FLOWFRAMEWORKS_FORBMVCFIGURE_HPP_
#define SRC_IMAGEALIGNMENT_FLOWFRAMEWORKS_FORBMVCFIGURE_HPP_

#include <stdio.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <DataTypes/Map.hpp>

//extern const string base;
//extern const string optimized_datasets;
//extern const string query_loc;
//extern const string siftloc;
//extern const string poses_loc;


class ForBMVCFigure{
private:

    Camera& _cam;
    std::vector<std::string>& _dates;
public:
    std::string _pftbase, _query_loc, _savebase, _visibility_dir, _map_base;
    
    ForBMVCFigure(Camera& cam, std::vector<std::string>& dates, std::string pftbase,
                  std::string query_loc, std::string results_dir):
    _cam(cam), _dates(dates), _pftbase(pftbase), _query_loc(query_loc), _savebase(results_dir + "aligned_images/"),
    _map_base(results_dir + "maps/")
    {}

	void GetAlignmentAtSection(std::string ref_date, int num, bool viewpoint_variance = false);
    
    void AlignSection(int num, std::string date1, std::string date2, int offset=0);

};
















#endif /* SRC_IMAGEALIGNMENT_FLOWFRAMEWORKS_FORBMVCFIGURE_HPP_ */
