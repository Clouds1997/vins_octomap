#pragma once

#include "ThirdParty/DBoW/QueryResults.h"
#include <map>
#include <vector>
#include <iostream>
#include <fstream>
#include <ostream>
#include <cmath>
#include <algorithm>
#include <stdio.h>
#include <string>

// using namespace std;
// using namespace DBoW2;

class NetVLAD {
private:
 std::map<unsigned int, std::vector<float>> database_;

public:
 NetVLAD();
 void query(const std::vector<float>& query, DBoW2::QueryResults& query_results,
            const int max_results, const int max_id) const;
 void add(const std::vector<float>& rep, const unsigned int id);
 double score(const std::vector<float>& d1, const std::vector<float>& d2) const;
};

