/*
 * PairwiseLineMatching.hh
 *
 *  Created on: 2011-8-19
 *      Author: lz
 */

#ifndef PAIRWISELINEMATCHING_HH_
#define PAIRWISELINEMATCHING_HH_
#include <map>
#include <opencv2/core/core.hpp>
#include "LineDescriptor.h"
#include <string>


//each node in the graph is a possible line matching pair in the left and right image
struct Node{
	unsigned int leftLineID;//the index of line in the left image
	unsigned int rightLineID;//the index of line in the right image
};

// Specifies a vector of nodes.
typedef std::vector<Node> Nodes_list;

struct CompareL {
    bool operator() (const double& lhs, const double& rhs) const
    {return lhs>rhs;}
};
typedef  std::multimap<double,unsigned int,CompareL> EigenMAP;
struct CompareS {
    bool operator() (const double& lhs, const double& rhs) const
    {return lhs<rhs;}
};
typedef  std::multimap<double,unsigned int,CompareS> DISMAP;
#define ResolutionScale 20 //10 degree
const unsigned int dim = 360 / ResolutionScale; //number of the bins of histogram

class PairwiseLineMatching
{
public:
    PairwiseLineMatching(){};
    void LineMatching(ScaleLines &linesInLeft,ScaleLines &linesInRight, std::vector<unsigned int> &matchResult);
    ~PairwiseLineMatching(){};
    void drawMatch(const cv::Mat &left, const ScaleLines &left_lines, 
        const cv::Mat &right, const ScaleLines &right_lines, 
        const std::vector<unsigned int> &matchResult, cv::Mat &show_image);
    double calAngleAndLenHist(ScaleLines &linesInLeft,  std::array<double, dim> &angleHist, std::array<double, dim> &lengthHist);
private:
    /* Compute the approximate global rotation angle between image pair(i.e. the left and right images).
   * As shown in Bin Fan's work "Robust line matching through line-point invariants", this approximate
   * global rotation angle can greatly prune the spurious line correspondences. This is the idea of their
   * fast matching version. Nevertheless, the approaches to estimate the approximate global rotation angle
   * are different. Their is based on the rotation information included in the matched point feature(such as SIFT)
   * while ours is computed from angle histograms of lines in images. Our approach also detect whether there is an
   * appropriate global rotation angle between image pair.
   * step 1: Get the angle histograms of detected lines in the left and right images, respectively;
   * step 2: Search the shift angle between two histograms to minimize their difference. Take this shift angle as
   *         approximate global rotation angle between image pair.
   * input:  detected lines in the left and right images
   * return: the global rotation angle
   */
    double GlobalRotationOfImagePair_(
        std::array<double, dim> angleHistLeft, std::array<double, dim> lengthLeft,
        std::array<double, dim> angleHistRight, std::array<double, dim> lengthRight);
    /* Build the symmetric non-negative adjacency matrix M, whose nodes are the potential assignments a = (i_l, j_r)
  * and whose weights on edges measure the agreements between pairs of potential assignments. That is where the pairwise
  * constraints are applied(c.f. A spectral technique for correspondence problems using pairwise constraints, M.Leordeanu).
  */
    void BuildAdjacencyMatrix_(ScaleLines &linesInLeft,ScaleLines &linesInRight) __attribute__ ((optimize(0))) ;
    /* Get the final matching from the principal eigenvector.
    */
    void MatchingResultFromPrincipalEigenvector_(ScaleLines &linesInLeft,ScaleLines &linesInRight,
    		std::vector<unsigned int > &matchResult);
    double globalRotationAngle_;//the approximate global rotation angle between image pairs

    /*construct a map to store the principal eigenvector and its index.
     *each pair in the map is in this form (eigenvalue, index);
     *Note that, we use eigenvalue as key in the map and index as their value.
     *This is because the map need be sorted by the eigenvalue rather than index
     *for our purpose.
      */
    EigenMAP eigenMap_;
    Nodes_list nodesList_;//save all the possible matched line pairs
    double minOfEigenVec_;//the acceptable minimal value in the principal eigen vector;
};




#endif /* PAIRWISELINEMATCHING_HH_ */
