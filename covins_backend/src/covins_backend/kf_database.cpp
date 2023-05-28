/**
* This file is part of COVINS.
*
* Copyright (C) 2018-2021 Patrik Schmuck / Vision for Robotics Lab
* (ETH Zurich) <collaborative (dot) slam (at) gmail (dot) com>
* For more information see <https://github.com/VIS4ROB-lab/covins>
*
* COVINS is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the license, or
* (at your option) any later version.
*
* COVINS is distributed to support research and development of
* multi-agent system, but WITHOUT ANY WARRANTY; without even the
* implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
* PURPOSE. In no event will the authors be held liable for any damages
* arising from the use of this software. See the GNU General Public
* License for more details.
*
* You should have received a copy of the GNU General Public License
* along with COVINS. If not, see <http://www.gnu.org/licenses/>.
*/
/**
* The KeyframeDatabase class partially re-uses code of ORB-SLAM2
*/

#include "covins_backend/kf_database.hpp"

// COVINS
#include "covins_backend/keyframe_be.hpp"

namespace covins {

KeyframeDatabase::KeyframeDatabase(const VocabularyPtr voc)
    : voc_(voc)
{
    inverted_file_index_.resize((*voc_).size());
    std::cout << "+++++ KeyFrame Database Initialized +++++" << std::endl;
}

auto KeyframeDatabase::AddKeyframe(KeyframePtr kf)->void {
    unique_lock<std::mutex> lock(mtx_);
    for(auto vit= kf->bow_vec_.begin(), vend=kf->bow_vec_.end(); vit!=vend; vit++)
        inverted_file_index_[vit->first].push_back(kf);
}

auto KeyframeDatabase::DetectCandidates(KeyframePtr kf, precision_t min_score)->KeyframeVector {
    KeyframeSet connected_kfs;
    KeyframeVector connections;
    if (covins_params::placerec::type == "COVINS_G") {
      connections = kf->GetConnectedNeighborKeyframes();
    } else {
      connections = kf->GetConnectedKeyframesByWeight(0);
    }
    

    connected_kfs.insert(connections.begin(),connections.end());
    KeyframeList lKFsSharingWords;

    // Search all keyframes that share a word with current keyframes
    // Discard keyframes that belong to KF's map
    {
        unique_lock<mutex> lock(mtx_);

        for(auto vit=kf->bow_vec_.begin(), vend=kf->bow_vec_.end(); vit != vend; vit++) {
            auto &lKFs = inverted_file_index_[vit->first];

            for(auto lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
            {
                KeyframePtr pKFi=*lit;
                if(pKFi->id_ == kf->id_) continue;
                if(covins_params::placerec::inter_map_matches_only && pKFi->id_.second == kf->id_.second) continue;
                if (pKFi->id_.second == kf->id_.second &&
                    abs(int(kf->id_.first) - int(pKFi->id_.first)) < covins_params::placerec::min_loop_dist) continue;
                if(pKFi->id_.first < covins_params::placerec::exclude_kfs_with_id_less_than) continue;
                if(!(pKFi->loop_query_==kf->id_)) {
                    pKFi->loop_words_=0;
                    if(!connected_kfs.count(pKFi)) {
                        pKFi->loop_query_=kf->id_;
                        lKFsSharingWords.push_back(pKFi);
                    }
                }
                pKFi->loop_words_++;
            }
        }
    }

    if(lKFsSharingWords.empty())
        return KeyframeVector();

    listFloatKfPair lScoreAndMatch;

    // Only compare against those keyframes that share enough words
    int maxCommonWords=0;
    for(KeyframeList::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        if((*lit)->loop_words_>maxCommonWords)
            maxCommonWords=(*lit)->loop_words_;
    }

    int minCommonWords = maxCommonWords*0.8f;

    int nscores=0;

    // Compute similarity score. Retain the matches whose score is higher than minScore
    for(KeyframeList::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        KeyframePtr pKFi = *lit;

        if(pKFi->loop_words_>minCommonWords)
        {
            nscores++;

            precision_t si = static_cast<precision_t>(voc_->score(kf->bow_vec_,pKFi->bow_vec_));

            pKFi->loop_score_ = si;
            if(si>=min_score)
                lScoreAndMatch.push_back(make_pair(si,pKFi));
        }
    }

    if(lScoreAndMatch.empty())
        return KeyframeVector();

    listFloatKfPair lAccScoreAndMatch;
    precision_t bestAccScore = min_score;

    // Lets now accumulate score by covisibility
    for(listFloatKfPair::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        KeyframePtr pKFi = it->second;
        KeyframeVector vpBestNeighs;
        KeyframeVector all_neighs;
        
        if (covins_params::placerec::type == "COVINS_G") {
          all_neighs = pKFi->GetConnectedNeighborKeyframes();
        } else {
          all_neighs = pKFi->GetConnectedKeyframesByWeight(0);   
        }

        if(all_neighs.size() < 10) vpBestNeighs = all_neighs;
        else vpBestNeighs = KeyframeVector(all_neighs.begin(),all_neighs.begin()+10);

        float bestScore = it->first;
        float accScore = it->first;
        KeyframePtr pBestKF = pKFi;
        for(KeyframeVector::iterator vit=vpBestNeighs.begin(), vend=vpBestNeighs.end(); vit!=vend; vit++)
        {
            KeyframePtr pKF2 = *vit;
            if(pKF2->loop_query_==kf->id_ && pKF2->loop_words_>minCommonWords)
            {
                accScore+=pKF2->loop_score_;
                if(pKF2->loop_score_>bestScore)
                {
                    pBestKF=pKF2;
                    bestScore = pKF2->loop_score_;
                }
            }
        }

        lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
        if(accScore>bestAccScore)
            bestAccScore=accScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    float minScoreToRetain = 0.75f*bestAccScore;

    KeyframeSet spAlreadyAddedKF;
    KeyframeVector vpLoopCandidates;
    vpLoopCandidates.reserve(lAccScoreAndMatch.size());

    for(listFloatKfPair::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
    {
        if(it->first>minScoreToRetain)
        {
            KeyframePtr pKFi = it->second;
            if(!spAlreadyAddedKF.count(pKFi))
            {
                vpLoopCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }

    return vpLoopCandidates;
}

auto KeyframeDatabase::EraseKeyframe(KeyframePtr kf)->void {
    unique_lock<std::mutex> lock(mtx_);
    // Erase elements in the Inverse File for the entry
    for(auto vit=kf->bow_vec_.begin(), vend=kf->bow_vec_.end(); vit!=vend; vit++) {
        // List of keyframes that share the word
        auto &list_kfs =   inverted_file_index_[vit->first];
        for(auto lit=list_kfs.begin(), lend= list_kfs.end(); lit!=lend; lit++) {
            if(kf==*lit) {
                list_kfs.erase(lit);
                break;
            }
        }
    }
}

} // end ns
