//
//  Cedric's code (adapted from ViSP), with ROS removed, and now in OpenCV.
//
//  This version created by Shane Griffith on 3/20/17.
//

#include "KLT.hpp"
#include <Visualizations/IMDraw.hpp>



void KLT::SetSizes(std::vector<double> tenparams){
     grid_rows = tenparams[0];
     grid_cols = tenparams[1];
     max_features = tenparams[2];
     max_features_per_cell = tenparams[3];
     max_feature_jump = tenparams[4];
     max_feature_dy = tenparams[5];
     m_winSize = tenparams[6];
     max_distance_fundamental = tenparams[7];
     m_blockSize = tenparams[8];   //pixel neighborhood block size
     m_minDistance = tenparams[9]; //min distance between features.
}

void KLT::reset_grids() {
    grid.clear();
    grid.resize(grid_rows*grid_cols);
    grid_new.clear();
    grid_new.resize(grid_rows*grid_cols);
}

bool KLT::recordKeypoint(int id, float x, float y) {
    KPMap::iterator it = all_keypoints.find(id);
    if (it == all_keypoints.end()) {
        all_keypoints.insert(KPMap::value_type(id, KPPath(1,cv::Point2f(x,y))));
    } else {
        cv::Point2f P = it->second[it->second.size()-1];
        cv::Point2f Pnew(x,y);
        if ((fabs(Pnew.y-P.y)>max_feature_dy)
            || (cv::norm(Pnew-P) > max_feature_jump)) {
            return false;
        }
        it->second.push_back(Pnew);
    }
    return true;
}

// Identify good matches using RANSAC
// Return fundemental matrix
size_t KLT::ransacTest() {
    size_t n_feat = (int)m_points[1].size();
    std::vector<cv::Point2f> points1;
    std::vector<cv::Point2f> points2; // prev
    // ROS_INFO("Running fundamental matrix filtering");
    for (size_t i=0;i<n_feat;i++) {
        KPMap::const_iterator it = all_keypoints.find(fid[i]);
        if ((it!=all_keypoints.end()) && (it->second.size()>1)) {
            // STRONG ASSUMPTION: for a given ID, points are inserted
            // every frame.
            KPPath::const_reverse_iterator pit = it->second.rbegin();
            points1.push_back(*pit);
            pit++;
            points2.push_back(*pit);
        }
    }
    std::vector<uchar> inliers(points1.size(),0);
    if (points1.size() && points2.size()) {
        cv::Mat fundemental= cv::findFundamentalMat(
                                                    cv::Mat(points1),cv::Mat(points2), // matching points
                                                    inliers,      // match status (inlier ou outlier)
                                                    CV_FM_RANSAC, // RANSAC method
                                                    max_distance_fundamental,     // distance to epipolar line
                                                    confidence_fundamental);  // confidence probability
    }
    
    size_t removed = 0;
    for(int i=inliers.size()-1; i>=0; i--) {
        if (!inliers[i]) {
            SuppressFeature(i);
            removed += 1;
        }
    }
    return removed;
}

void KLT::SuppressFeature(int index){
    if ((size_t)index >= m_points[1].size()){
        std::cout<< "Feature ["<< index <<"] doesn't exist" << std::endl;
        exit(-1);
    }
    
    all_points.erase(fid[index]);
    all_keypoints.erase(fid[index]);
    fid.erase(fid.begin()+index);
    m_points[1].erase(m_points[1].begin()+index);
    m_points[0].erase(m_points[0].begin()+index); //should be unnecessary.
}

void KLT::InitializeTracking(cv::Mat& view) {
    for (size_t i=0; i<2; i++) m_points[i].clear();
    
    cv::goodFeaturesToTrack(view, m_points[1], max_features, m_qualityLevel, m_minDistance, cv::Mat(), m_blockSize, 0, m_harris_k);
    
    fid.clear();
    std::vector<cv::Point2f> feat = m_points[1];
    size_t n_feat = (int) feat.size();
    if(m_points[1].size() > 0){
        cv::cornerSubPix(view, m_points[1], cv::Size(m_winSize, m_winSize), cv::Size(-1,-1), m_termcrit);
        
        for (size_t i=0; i < n_feat; i++)
            fid.push_back(m_next_points_id++);
    }
    
    maxid = m_next_points_id-1;
    for (size_t i=0; i<n_feat; i++) {
        feature_positions fp = {(int) fid[i], feat[i].x, feat[i].y};
        recordKeypoint(fp.id, fp.x, fp.y);
        all_points[fp.id] = fp;
    }

    view.copyTo(prev_view);
}

bool KLT::track(cv::Mat &view) {
    //track the set of known points using pyramidal LK optical flow
    if(m_points[1].size() == 0){
        std::cout << "Not enough key points to track." << std::endl;
        return false;
    }
    
    if(prev_view.empty()) view.copyTo(prev_view);
    std::swap(m_points[1], m_points[0]);
    std::vector<uchar> status;
    std::vector<float> erroutput;
    cv::calcOpticalFlowPyrLK(prev_view, view, m_points[0], m_points[1], status, erroutput, cv::Size(m_winSize, m_winSize), m_pyrMaxLevel, m_termcrit, 0, 1e-4);
    
    // Remove lost points
    for(int i=(int)status.size()-1; i>=0; i--) {
        if(status[i] == 0)
            SuppressFeature(i);
    }
    
    view.copyTo(prev_view);
    return true;
}

void KLT::ContinueTracking(cv::Mat& view){
    if(!track(view)) {
        InitializeTracking(view);
        return;
    }
    
    std::vector<cv::Point2f> new_points;
    double grid_delta_x = view.cols / (float) grid_cols;
    double grid_delta_y = view.rows / (float) grid_rows;
    cv::goodFeaturesToTrack(view, new_points, max_features, 0.01,
                            std::min(grid_delta_x, grid_delta_y)/2, cv::Mat(), 3, 0, 0.04);
    
    reset_grids();
    for (size_t i=0; i<new_points.size(); i++) {
        int jg = floor(std::max(0.0f,std::min((float) (new_points[i].x / grid_delta_x), grid_cols-1.0f)));
        int ig = floor(std::max(0.0f,std::min((float) (new_points[i].y / grid_delta_y), grid_rows-1.0f)));
        grid_new[ig*grid_cols + jg].push_back(i);
    }
    
    int removed = 0;
    std::vector<cv::Point2f>& feat = m_points[1];//SAVE. keep the '&' here.
    size_t n_feat = (int) feat.size();
    for (int i=n_feat-1; i>=0; i--) {
        if (!recordKeypoint(fid[i], feat[i].x, feat[i].y)) {
            SuppressFeature(i);
            removed += 1;
        }
    }
    
    if (filter_with_fundamental) removed += ransacTest();
    
    n_feat = (int) m_points[1].size();
    long minid = n_feat?fid[0]:-1;
    for (size_t i=0; i<n_feat; i++) {
        //minid = std::min(fid[i],minid);
        if(fid[i] < minid) {minid = fid[i]; if(i !=0 ) std::cout<<"min other than the first entry"<<std::endl;}
        int jg = floor(std::max(0.0f, std::min((float) (feat[i].x / grid_delta_x), grid_cols-1.0f)));
        int ig = floor(std::max(0.0f, std::min((float) (feat[i].y / grid_delta_y), grid_rows-1.0f)));
        grid[ig*grid_cols + jg].push_back(i);
    }
    
    //suppress points that exceed the max number of features per cell.
    std::vector<long> to_suppress;
    for (size_t i=0; i<grid.size(); i++) {
        if (grid[i].size()>(size_t)max_features_per_cell) {
            for (std::vector<long>::const_reverse_iterator j=grid[i].rbegin();j!=grid[i].rend();j++) {
                to_suppress.push_back(*j);
                removed += 1;
            }
            grid[i].clear();
        }
    }
    
    std::sort(to_suppress.begin(), to_suppress.end());
    for (std::vector<long>::const_reverse_iterator j=to_suppress.rbegin();
         j!=to_suppress.rend(); j++) {
        SuppressFeature(*j);
    }
    
    std::vector<long> to_consider;
    for (size_t i=0; i<grid.size(); i++) to_consider.push_back(i);
    std::random_shuffle(to_consider.begin(), to_consider.end());
    
    std::vector<long> to_sift;
    for (size_t i=0; i<to_consider.size(); i++) {
        if (grid[to_consider[i]].size()>0) continue;
        if (int(m_points[1].size()+to_sift.size()) >= max_features) break;
        size_t N = std::min(grid_new[to_consider[i]].size(), (size_t)max_features_per_cell);
        for (size_t j=0; j<N; j++) {
            if (int(m_points[1].size()+to_sift.size()) >= max_features) break;
            to_sift.push_back(grid_new[to_consider[i]][j]);
        }
    }
    
    n_feat = (int) m_points[1].size();
    int added = 0;
    for (size_t i=0; i<n_feat + to_sift.size(); i++) {
        if(i < n_feat) {
            all_points.erase(fid[i]);
            feature_positions fp = {(int) fid[i], feat[i].x, feat[i].y};
            all_points[fp.id] = fp;
        } else {
            maxid += 1;
            feature_positions fp = {(int) maxid, new_points[to_sift[added]].x, new_points[to_sift[added]].y};
            AddFeature(fp.id, fp.x, fp.y);
            recordKeypoint(fp.id, fp.x, fp.y);
            all_points[fp.id] = fp;
            added += 1;
        }
    }
    
    printf("maxid %d | %d features: Added %d features, removed %d\n", (int) maxid, (int) m_points[1].size(), added, removed);
}

void KLT::AddFeature(long id, float x, float y) {
    cv::Point2f f(x, y);
    m_points[1].push_back(f);
    fid.push_back(id);
    if (id >= m_next_points_id)
        m_next_points_id = id + 1;
}

std::vector<KLT::feature_positions> KLT::UpdateCurrentPointSet() {
    size_t n_feat = (int)m_points[1].size();
    std::vector<KLT::feature_positions> feats(n_feat);
    for (size_t i=0; i<n_feat; i++) {
        KLT::feature_positions fp = {(int) fid[i], -1., -1.};
        feats[i] = fp;
        KLTMap::const_iterator it = all_points.find(fid[i]);
        if (it != all_points.end())
            feats[i] = it->second;
        if(i>60 && i<=65)std::cout << feats[i].id<<": "<<feats[i].x <<", "<< feats[i].y << std::endl;
    }

    return feats;
}

void KLT::PrintPointSet() {
    char tmp[128];
    sprintf(tmp,"KLTPoint%04d.log", print_feature);
    FILE *fp = fopen(tmp,"a");
    for (int id=0; id <= print_feature; id++) {
        KLTMap::const_iterator it = all_points.find(id);
        if (it != all_points.end()) {
            const KLT::feature_positions & P = it->second;
            printf("KLT Point %d at %.2f %.2f\n", P.id, P.x, P.y);
            fprintf(fp,"%d %e %e\n", P.id, P.x, P.y);
        }
    }
    fclose(fp);
}

ParseFeatureTrackFile KLT::TrackKLTFeatures(cv::Mat& view_color, std::string base, int image_num, double timestamp) {
    cv::Mat view;
    cvtColor(view_color, view, CV_RGB2GRAY);
    
    ParseFeatureTrackFile PFT(_cam);
    ContinueTracking(view);

    if (print_feature>=0) {
        PrintPointSet();
    }
    
    std::vector<feature_positions> feats = UpdateCurrentPointSet();
    std::vector<int> ids(feats.size(), 0);
    std::vector<cv::Point2f> points(feats.size());
    for(int i=0; i<ids.size(); i++){
        ids[i] = feats[i].id;
        points[i] = cv::Point2f(feats[i].x, feats[i].y);
    }

    //return the PFT
    PFT.SetPFTContents(base, image_num, timestamp, ids, points);
    return PFT;
}

void KLT::DrawFeatures(cv::Mat& image){
    IMDraw art(image);
    for(int i=0; i<fid.size(); i++)
        art.DrawPoint(m_points[1][i].x, m_points[1][i].y, fid[i]);
}





