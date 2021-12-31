#include "pl_block.h"
#include "opencv_utils.h"
//按照五个像素大小进行聚类
void PLStructure::nonMaxminuSuppression() {
    for (size_t row = 0; row < rows; ++row) {
        for (size_t col = 0; col < cols; ++col) {
            auto &_block = block[row][col];
            auto &hlevel_pts = _block.high_level_pts;
            for (size_t p = 0; p < hlevel_pts.size(); ++p) {
                if (scale_pts.hierarchy[hlevel_pts[p]].empty()) {
                    hlevel_pts.erase(hlevel_pts.begin() + p--);
                }
            }
        }
    }
}


//1，先建立分块存储索引
//2，再按照分块索引计算上下级关系
//hierarchy关系在每块内计算
void PLStructure::loadData(const std::vector<cv::KeyPoint> &key_pts, const ScaleLines &lines) {
    scale_pts.keypoints = key_pts;
    scale_lines = lines;
    block_w = image.cols / cols;
    block_h = image.rows / rows;
    grid_w = image.cols / GRID_COLS;
    grid_h = image.rows / GRID_ROWS;
    block.resize(rows);
    // scale_pts.points.resize(max_octave);
    int b_row;
    int b_col;
    int kypt_size = key_pts.size();
    int line_size = scale_lines.size();
    int pts_per_oct = key_pts.size() / max_octave;
    scale_pts.hierarchy.resize(kypt_size);
    for (auto &_block : block) {
        _block.resize(cols);
        for (auto &_block_:_block) {
            _block_.scale_pts.resize(max_octave);
        }
    }
    // for (auto &points : scale_pts.points) {
    //     points.reserve(pts_per_oct);
    // }
    for (int i = 0; i < kypt_size; ++i) {
        auto const &kpt = key_pts[i];
        // scale_pts.points[kpt.octave].push_back(kpt);
        b_row = kpt.pt.y / block_h;
        b_col = kpt.pt.x / block_w;
        block[b_row][b_col].scale_pts[kpt.octave].push_back(i);
        if (kpt.octave < low_level) {
            block[b_row][b_col].low_level_pts.push_back(i);
            // GRID
            int grid_row = kpt.pt.y / grid_h;
            int grid_col = kpt.pt.x / grid_w;
            grid_row = grid_row < 0 ? 0 : grid_row;
            grid_col = grid_col < 0 ? 0 : grid_col;
            grid_row = grid_row >= GRID_ROWS ? GRID_ROWS - 1 : grid_row;
            grid_col = grid_col >= GRID_COLS ? GRID_COLS - 1 : grid_col;
            plGrid[grid_row][grid_col].push_back(i);
        } else {
            block[b_row][b_col].high_level_pts.push_back(i);
        }
    }
    //线段应该进行采样，重新设置可见性，采样点根据变换关系进行对其匹配，恢复线段位姿
    for (int i = 0; i < line_size; ++i) {
        for (const auto &pt : scale_lines[i][0].keypoints) {
            b_row = pt.pt.y / block_h;
            b_col = pt.pt.x / block_w;
            block[b_row][b_col].scale_lines.insert(i);
        }
    }
    
    //parallel for  
    //搜索临近八个block计算size内包含的底层特征点
    //从底层开始向上搜索，直接归入下一级
    //保证大尺度互不重叠，存为一个ocave，底层特征点合并为一个尺度

    //记录是否被分配  现在直接删除index， 但是erase时间复杂度比较高？
    // bool distributed[kypt_size];
    // memset(distributed, 0, sizeof(bool) * kypt_size);
    for (int blockR = 0; blockR < block.size(); ++blockR) {
        for (int blockC = 0; blockC < block[blockR].size(); ++blockC) {//loop for block
            block[blockR][blockC].row = blockR;
            block[blockR][blockC].col = blockC;
            // std::vector<int> low_level_pts = {};
            //搜索周围九个栅格，合并low_level_pts indices
            for (int i = -1; i < 2; ++i) {
                for (int j = -1; j < 2; ++j) {
                    int blockRI = blockR + i;
                    int blockCI = blockC + j;
                    if (blockRI < 0 || blockRI >= rows || blockCI < 0 || blockCI >= cols) continue;
                    auto &_block = block[blockRI][blockCI];
                    // auto const &block_low = block[blockRI][blockCI].low_level_pts;
                    // low_level_pts.insert(low_level_pts.end(), block_low.begin(), block_low.end());     


                    for (int hlevelId = 0; hlevelId < _block.high_level_pts.size(); ++hlevelId) {//loop for big scale points
                        int hlevelid_in_keypts = _block.high_level_pts[hlevelId];
                        if (hlevelid_in_keypts >= scale_pts.points.size()) continue;
                        auto const &hlevelPt = key_pts[hlevelid_in_keypts];
                        // auto const &pts_indices = _block.low_level_pts;
                        for (int lovelId = 0; lovelId < _block.low_level_pts.size(); ++lovelId) {//loop for low level points
                            auto const &lovePt = key_pts[_block.low_level_pts[lovelId]];
                            if (std::abs(lovePt.pt.x - hlevelPt.pt.x) < hlevelPt.size && std::abs(lovePt.pt.x - hlevelPt.pt.x) < hlevelPt.size) {
                                //存放包含底层特征索引
                                scale_pts.hierarchy[_block.high_level_pts[hlevelId]].push_back(_block.low_level_pts[lovelId]);
                                //底层存放所属高层索引
                                scale_pts.hierarchy[_block.low_level_pts[lovelId]].push_back(_block.high_level_pts[hlevelId]);
                                //删除已经分配的底层特征并更新 for 索引
                                _block.low_level_pts.erase(_block.low_level_pts.begin() + lovelId--);
                            }
                            
                        }
                        
                    }
                }   
            }
        }
    }
}


//to do改成从未计算的点生长
void PLStructure::spreadLines() {
    // std::vector<cv::Point> boundary = {};
    // bool visited[rows * cols];
    // memset(visited, 0, sizeof(bool) * rows * cols);
    // for (size_t row = 0; row < rows; ++row) {
    //     for (size_t col = 0; col < cols; ++col) {
    //         auto &_block_ = block[row][col];
    //         if (!_block_.calculated) continue;
    //         boundary.push_back(cv::Point(col, row));
    //         visited[row * cols + row] = true;       
    //     }
    // }
    // while (true) {
    //     std::vector<cv::Point> new_boundary = {};
    //     for (const auto &pt : boundary) {
    //         for (int i = -1; i < 2; ++i) {
    //             for (int j = -1; j < 2; ++j) {
    //                 int rowi = pt.y + i;
    //                 int colj = pt.x + j;
    //                 if (rowi < 0 || rowi >= rows || colj < 0 || colj >= cols) continue;
    //                 if (visited[rowi * cols + colj]) continue;
    //                 new_boundary.push_back(colj, rowi);
    //                 block[rowi][colj].calculated = true;
    //             }
    //         }
    //     }
    //     if (new_boundary.empty()) break;
    //     boundary = std::move(new_boundary);
    // }
    // std::vector<cv::Point> boundary = {};
    std::vector<cv::Point> empty_block = {};
    bool empty[rows * cols];
    memset(empty, 0, sizeof(bool) * rows * cols);//设置为0， 都是非空
    for (size_t row = 0; row < rows; ++row) {
        for (size_t col = 0; col < cols; ++col) {
            auto &_block_ = block[row][col];
            if (!_block_.scale_lines.empty()) continue;
            empty_block.push_back(cv::Point(col, row));
            empty[row * cols + col] = true;       
        }
    }
    for (const auto& pt : empty_block) {
        auto &_block_ = block[pt.y][pt.x];
        typedef std::pair<cv::Point, double> dis_pair;
        auto cmp = [](dis_pair l, dis_pair r) { return l.second > r.second;};
        std::priority_queue<dis_pair, std::vector<dis_pair>, decltype(cmp)> dis_pri(cmp);
        for (size_t row = 0; row < rows; ++row) {
            for (size_t col = 0; col < cols; ++col) {
                if (empty[row * cols + col]) continue;
                dis_pri.push(dis_pair(cv::Point(col, row), (col - pt.x) * (col - pt.x) + (row - pt.y) * (row - pt.y)));
            }
        }
        while (true) {
            dis_pair pair = dis_pri.top();
            auto const &lines = block[pair.first.y][pair.first.x].scale_lines;
            _block_.scale_lines.insert(lines.begin(), lines.end());
            dis_pri.pop();
            if (pair.second < dis_pri.top().second) break;
        }
    }
    
    // for (int i = 0; i < empty_block.size(); ++i) {
    // for (const auto& pt : empty_block) {
    //     auto &_block_ = block[pt.y][pt.x];
    //     boundary.clear();
    //     boundary.push_back(pt);
    //     while (_block_.scale_lines.size() < 2) {
    //         std::vector<cv::Point> new_boundary = {};
    //         for (const auto &bpt : boundary) {
    //             for (int i = -1; i < 2; ++i) {
    //                 for (int j = -1; j < 2; ++j) {
    //                     int rowi = bpt.y + i;
    //                     int colj = bpt.x + j;
    //                     if (rowi < 0 || rowi >= rows || colj < 0 || colj >= cols) continue;
    //                     new_boundary.push_back(colj, rowi);
    //                     block[rowi][colj].calculated = true;
    //                 }
    //             }
    //         }
            
    //     }
        
    // }
    
    
}


//只计算有匹配正确大尺度点的块，再生长到没有的地方
void PLStructure::computeLocalTransform(
    const PLStructure &right_structure, std::vector<int> _line_match, std::vector<cv::DMatch> _pt_match) {
    
    // spreadLines();
    
    int linem_size = _line_match.size() / 2;
    int point_size = _pt_match.size();
    int *ptr = line_match.data();
    line_match = std::move(std::vector<int>(scale_lines.size(), -1));
    point_match = std::move(std::vector<int>(scale_pts.keypoints.size(), -1));
    for (size_t i = 0; i < linem_size; ++i) {
        line_match[_line_match[i * 2]] = _line_match[i * 2 + 1];
    }
    for (const auto & ptmatch : _pt_match) {
        point_match[ptmatch.queryIdx] = point_match[ptmatch.trainIdx];
    }
    int b_row;
    int b_col;
    //首先计算每个大尺度角点和最近线段的变换关系，再生长到附近没有的地方，或者插值，根据生长距离确定预测矩形框大小
    for (const auto &match : _pt_match) {
        int left_id = match.queryIdx;
        int right_id = match.trainIdx;
        const float &x1 = this->scale_pts.keypoints[left_id].pt.x;
        const float &y1 = this->scale_pts.keypoints[left_id].pt.y;
        const float &x2 = right_structure.scale_pts.keypoints[right_id].pt.x;
        const float &y2 = right_structure.scale_pts.keypoints[right_id].pt.y;
        // double dx = right_structure.scale_pts.keypoints[left_id].pt.x - this->scale_pts.keypoints[right_id].pt.x;
        // double dy = right_structure.scale_pts.keypoints[left_id].pt.y - this->scale_pts.keypoints[right_id].pt.y;
        //寻找最近的线段，计算
        b_row = y1 / block_h;
        b_col = x1 / block_w;
        auto &_block_ = block[b_row][b_col];
        if (_block_.calculated) continue;
        
        std::set<int> line_set = _block_.scale_lines;
        // while (true) {
        //     if (line_set.size() > 2) break;
        //     //区域生长

        // }
        //to do
        //随便取一个线段，后面改成最近的，最长，最显著的
        double angle = -100;
        for (auto line_id : line_set) {
            const auto &lineleft = scale_lines[line_id][0]; 
            if (line_match[line_id] < 0) continue;
            const auto &lineright = right_structure.scale_lines[line_match[line_id]][0]; 
            double tmp_angle = lineright.direction - lineleft.direction;
            // if (tmp_angle > 2 * M_PI) {
            //     tmp_angle -= 2 * M_PI;
            // }
            // tmp_angle = tmp_angle ;
            angle = tmp_angle;
            break;
        }
        if (angle == -100) {
            continue;
        }
        
        _block_.transform = cv::getRotationMatrix2D(cv::Point(0, 0), angle, 1.0);
        // const auto &_transform = _block_.transform;
        // _block_.transform.at<float>(0, 2) = dx;
        // _block_.transform.at<float>(1, 2) = dy;
        // rotation.copyTo(_block_.transform.colRange(0, 3).rowRange(0, 2));
        const auto &_ptr = _block_.transform.ptr<double>();
        _block_.transform.at<double>(0, 2) = x2 - (_ptr[0] * x1 + _ptr[1] * y1);
        _block_.transform.at<double>(1, 2) = y2 - (_ptr[3] * x1 + _ptr[4] * y1);
        _block_.calculated = true;
        std::cout << _block_.transform << "\n";
        
    }
    
    //生长到其他地方
    for (const auto &_block : block) {
        for (const auto &_block_ : _block) {
            
        }
    }
    
}

void PLStructure::spreadTransform() {
    std::vector<cv::Point> empty_block = {};
    bool empty[rows * cols];
    memset(empty, 0, sizeof(bool) * rows * cols);//设置为0， 都是非空
    for (size_t row = 0; row < rows; ++row) {
        for (size_t col = 0; col < cols; ++col) {
            auto &_block_ = block[row][col];
            if (_block_.calculated) continue;
            empty_block.push_back(cv::Point(col, row));
            empty[row * cols + col] = true;       
        }
    }

    for (const auto& pt : empty_block) {
        auto &_block_ = block[pt.y][pt.x];
        typedef std::pair<cv::Point, double> dis_pair;
        auto cmp = [](dis_pair l, dis_pair r) { return l.second > r.second;};
        std::priority_queue<dis_pair, std::vector<dis_pair>, decltype(cmp)> dis_pri(cmp);
        for (size_t row = 0; row < rows; ++row) {
            for (size_t col = 0; col < cols; ++col) {
                if (empty[row * cols + col]) continue;
                dis_pri.push(dis_pair(cv::Point(col, row), (col - pt.x) * (col - pt.x) + (row - pt.y) * (row - pt.y)));
            }
        }
        //to do改成angle 和dx dy均值再求transform
        while (true) {
            dis_pair pair = dis_pri.top();
            auto const &_transform = block[pair.first.y][pair.first.x].transform;
            _block_.transform = _transform.clone();
            dis_pri.pop();
            if (pair.second < dis_pri.top().second) break;
        }
    }
}


void PLStructure::test() {

    // spreadLines();

    const auto &key_pts = scale_pts.keypoints;
    cv::Mat show_image;
    cv::cvtColor(image, show_image, cv::COLOR_GRAY2BGR);
    for (int blockR = 0; blockR < block.size(); ++blockR) {
        for (int blockC = 0; blockC < block[blockR].size(); ++blockC) {
            auto const &_block = block[blockR][blockC];
            //画大尺度点
            auto const &hlevel_pt = _block.high_level_pts;
            for (int hlevelId = 0; hlevelId < hlevel_pt.size(); ++hlevelId) {//loop for big scale points
                auto const &hlevelPt = key_pts[hlevel_pt[hlevelId]];
                auto const &love_pts = scale_pts.hierarchy[hlevel_pt[hlevelId]];
                // if (love_pts.empty()) continue;
                // if (hlevelPt.octave < 7) continue;
                if (hlevel_pt[hlevelId] >= scale_pts.points.size()) continue;
                
                cv::Scalar color = tergeo::visualodometry::randColor();
                cv::circle(show_image, hlevelPt.pt, hlevelPt.size / 2, color);
                auto const &pts_indices = _block.low_level_pts;
                for (int lovelId = 0; lovelId < love_pts.size(); ++lovelId) {//loop for low level points
                    auto const &lovePt = key_pts[love_pts[lovelId]];
                    cv::circle(show_image, lovePt.pt, lovePt.size / 2, color);
                    
                }
                
            }
        }
    }
    cv::imshow("dis", show_image);
    cv::waitKey();
    for (const auto &line_vec : scale_lines) {
        for (const auto &line : line_vec) {
            line.draw(show_image);
            // cv::line(show_image, cv::Point(line.sPointInOctaveX, line.sPointInOctaveY), 
            //         cv::Point(line.ePointInOctaveX, line.ePointInOctaveY), tergeo::visualodometry::red);
            // for (const auto &pt : line.keypoints) {
            //     cv::circle(show_image, pt.pt, 5, tergeo::visualodometry::green);
            // }
            
        }
    }

    //draw block
    for (const auto &_block : block) {
        for (const auto &_block_ : _block) {
            cv::cvtColor(image, show_image, cv::COLOR_GRAY2BGR);
            int ox = _block_.col * block_w;
            int oy = _block_.row * block_h;
            cv::line(show_image, cv::Point(ox, oy), cv::Point(ox + block_w, oy), tergeo::visualodometry::green);
            cv::line(show_image, cv::Point(ox + block_w, oy), cv::Point(ox + block_w, oy + block_h), tergeo::visualodometry::green);
            cv::line(show_image, cv::Point(ox + block_w, oy + block_h), cv::Point(ox, oy + block_h), tergeo::visualodometry::green);
            cv::line(show_image, cv::Point(ox, oy + block_h), cv::Point(ox, oy), tergeo::visualodometry::green);
            const auto &line_ids = _block_.scale_lines;
            for (const auto &id : line_ids) {
                scale_lines[id][0].draw(show_image);
            }
            cv::imshow("dis", show_image);
            cv::waitKey();
        }
    }
    
    
    cv::imshow("dis", show_image);
    cv::waitKey();
}

std::vector<int> PLStructure::GetFeaturesInArea(
        const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel, 
        const std::vector<int> &match_indices) {
    std::vector<int> vIndices;
    // vIndices.reserve(N);

    int grid_row = y / grid_h;
    int grid_col = x / grid_w;
    grid_row = grid_row < 0 ? 0 : grid_row;
    grid_col = grid_col < 0 ? 0 : grid_col;
    grid_row = grid_row >= GRID_ROWS ? GRID_ROWS - 1 : grid_row;
    grid_col = grid_col >= GRID_COLS ? GRID_COLS - 1 : grid_col;
    
    static const std::vector<std::pair<int, int>> grid_offset = {{-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 0}, {0, 1}, {1, -1}, {1, 0}, {1, 1}};
    
    for (size_t block_i = 0; block_i < 9; ++block_i) {
        int col_i = grid_col + grid_offset[block_i].first;
        int row_i = grid_row + grid_offset[block_i].second;
        if (col_i < 0 || col_i >= GRID_COLS || row_i < 0 || row_i >= GRID_ROWS) continue;
        auto const &pt_indices = plGrid[row_i][col_i];
        for (size_t i = 0; i < pt_indices.size(); ++i) {
            if (match_indices[pt_indices[i]] > -2) continue;

            vIndices.push_back(pt_indices[i]);
            
        }
    }
    
    return vIndices;
}

void PLStructure::computeDescriptor(cv::Ptr<cv::Feature2D> feature_detector) {
    std::vector<cv::KeyPoint> isloted_keypt = {};
    for (size_t i = 0; i < isolate_pts.size(); ++i) {
        if (isolate_pts[i] < 0) continue;
        isloted_keypt.push_back(scale_pts.keypoints[i]);
    }
    scale_pts.descriptor.release();
    feature_detector->compute(image, isloted_keypt, scale_pts.descriptor);
    // feature_detector->detectAndCompute(image, cv::noArray(), isloted_keypt, scale_pts.descriptor, true);
}

double PLStructure::computeDistance(const cv::Mat &left, const cv::Mat &right) {
    float * left_ptr = (float *)left.ptr();
    float * right_ptr = (float *)right.ptr();
    float dis = 0, temp;
    for (int i = 0; i < scale_pts.dim_of_descriptor; ++i) {
        temp = *left_ptr++ - *right_ptr++; //discriptor minus save to temp
        dis += temp * temp;
    }
    return dis;
}

cv::Point2f PLStructure::transformPoint(const cv::Point2f &pt) {
    cv::Point2f out_put; 
    int b_row = pt.y / block_h;
    int b_col = pt.x / block_w;
    double * _ptr = block[b_row][b_col].transform.ptr<double>();
    std::cout << block[b_row][b_col].transform << "\n";
    std::cout << block[b_row][b_col].transform.type() << "\n";
    out_put.x = _ptr[0] * pt.x + _ptr[1] * pt.y + _ptr[2];
    out_put.y = _ptr[3] * pt.x + _ptr[4] * pt.y + _ptr[5];
    return out_put;
}

void PLStructure::computeAngleInIsolatedPoint(bool right) {
    // int 
    // if (right) {
    //     for (size_t i = 0; i < isolate_pts; i++)
    //     {
    //         /* code */
    //     }
        
    // }
    
}


