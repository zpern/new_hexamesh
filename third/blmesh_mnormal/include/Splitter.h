
#pragma once
#ifndef _SPLITTER_H_
#define _SPLITTER_H_
#include "singleton.h"
#include <vector>
#include <set>
#include <map>
class Splitter:public Singleton<Splitter>
{
public:
	std::vector<std::set<int>> getSpliiter(std::set<int> input) {
		if (inner_map.find(static_cast<int>(input.size()))==inner_map.end()) {
			// at least 2 splitters, and at most input.size() splitters
			for (int i = 2; i <= input.size(); i++) {

				if (inner_map[i].empty()) {
					if (i == 2) {
						inner_map[i].push_back(std::vector<int>{0, 1});
						inner_map[i].push_back(std::vector<int>{0});
						inner_map[i].push_back(std::vector<int>{1});
						inner_map[i].push_back(std::vector<int>{});
					}
					else
					{						
						inner_map[i] = inner_map[i - 1];
						for (int k = 0; k < inner_map[i - 1].size(); k++) {
							vector<int> t = inner_map[i - 1][k];
							t.push_back(i-1);
							inner_map[i].push_back(t);
						}
					}
				}
			}		
		}
		
		std::vector<std::set<int>> ans;
		std::vector<int> indexs;
		for (auto i : input)
			indexs.push_back(i);


		for (auto i : inner_map[int(input.size())]) {

			std::set<int> one;
			if (i.size() < 2)
				continue;
			for (auto j : i) {
				//cout << j << " ";
				one.insert(indexs[j]);
			}
			//cout << endl;
			ans.push_back(one);
		}

		return ans;

	}
protected:
	std::map<int, std::vector<std::vector<int>>> inner_map;
}; 



#endif

