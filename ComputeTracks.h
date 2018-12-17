//
// Created by GING on 2018-12-15.
//
#include <set>

#include "UnionFind.h"
#include "Definitions.h"
#include "flat_pair_map.h"

#ifndef INCREMENTAL_SFM_COMPUTETRACKS_H
#define INCREMENTAL_SFM_COMPUTETRACKS_H

//Compute Initial tracks
//this function will initial uf_tree,so you should enter an empty uf_tree
//tracks will be saved in uf_tree
void Compute_Tracks(UnionFind &uf_tree, MAP_MATCHES & in_MapMatches,flat_pair_map<pair<int,int>,int> & out_map_node_to_index);


//Remove tracks with more than one feature in one image
//Remove tracks if too short
//tracks will be exported from uf_tree to out_tracks
void Filter_Tracks(UnionFind & in_ufTree,flat_pair_map<pair<int,int>,int> & in_map_node_to_index,MAP_TRACKS & out_tracks);


#endif //INCREMENTAL_SFM_COMPUTETRACKS_H
