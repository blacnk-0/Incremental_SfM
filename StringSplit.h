//
// Created by GING on 2018-12-19.
//

#include <string>
#include <vector>

using namespace std;

#ifndef INCREMENTAL_SFM_STRINGSPLIT_SELFMADE_H
#define INCREMENTAL_SFM_STRINGSPLIT_SELFMADE_H

vector<string> split_by_character(const string & str,char sep)
{
    vector<string> split_collection;

    if(str.empty())
    {
        return split_collection;
    }

    string::size_type  start{0};
    string::size_type  current_index=str.find(sep,start);
    while(current_index!=string::npos)
    {
        string str_sep=str.substr(start,current_index-start);
        split_collection.push_back(str_sep);
        start=current_index+1;
        current_index=str.find(sep,start);
    }

    split_collection.push_back(str.substr(start));
    return split_collection;

}

#endif //INCREMENTAL_SFM_STRINGSPLIT_SELFMADE_H
