//
// Created by GING on 2018-12-19.
//

#include <string>
#include <vector>


#ifndef INCREMENTAL_SFM_STRINGSPLIT_SELFMADE_H
#define INCREMENTAL_SFM_STRINGSPLIT_SELFMADE_H

std::vector<std::string> split_by_character(const std::string & str,char sep)
{
    std::vector<std::string> split_collection;

    if(str.empty())
    {
        return split_collection;
    }

    std::string::size_type  start{0};
    std::string::size_type  current_index=str.find(sep,start);
    while(current_index!=std::string::npos)
    {
        std::string str_sep=str.substr(start,current_index-start);
        split_collection.push_back(str_sep);
        start=current_index+1;
        current_index=str.find(sep,start);
    }

    split_collection.push_back(str.substr(start));
    return split_collection;

}

#endif //INCREMENTAL_SFM_STRINGSPLIT_SELFMADE_H
