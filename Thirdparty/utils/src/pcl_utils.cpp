#include "pcl_utils.h"

// #include <pcl/filters/radius_outlier_removal.h>
// #include <pcl/filters/conditional_removal.h>

namespace tergeo{
namespace visualodometry {

// typedef pcl::PointXYZ PoinT;

// int *rand_rgb(){//随机产生颜色
// 	int *rgb = new int[3];	
// 	rgb[0] = rand() % 255;
// 	rgb[1] = rand() % 255;
// 	rgb[2] = rand() % 255;
// 	return rgb;
// }

// pcl::PolygonMesh pointCloudGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr pcloud) {
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downSampled(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
// 	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed(new pcl::PointCloud<pcl::PointXYZ>);
//     //jiangcaiyang
//     pcl::VoxelGrid<pcl::PointXYZ> downSampled;  //创建滤波对象
//     downSampled.setInputCloud (pcloud);            //设置需要过滤的点云给滤波对象
//     downSampled.setLeafSize (0.1f, 0.1f, 0.1f);  //设置滤波时创建的体素体积为1cm的立方体
//     downSampled.filter (*cloud_downSampled);
//     // 统计滤波
//      pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statisOutlierRemoval;       //创建滤波器对象
//     statisOutlierRemoval.setInputCloud (cloud_downSampled);            //设置待滤波的点云
//     statisOutlierRemoval.setMeanK (50);                                //设置在进行统计时考虑查询点临近点数
//     statisOutlierRemoval.setStddevMulThresh (3.0);                     //设置判断是否为离群点的阀值:均值+1.0*标准差
//     statisOutlierRemoval.filter (*cloud_filtered);  

//     // 对点云重采样  
//     pcl::search::KdTree<pcl::PointXYZ>::Ptr treeSampling (new pcl::search::KdTree<pcl::PointXYZ>);// 创建用于最近邻搜索的KD-Tree
//     pcl::PointCloud<pcl::PointXYZ> mls_point;    //输出MLS
//     pcl::MovingLeastSquares<pcl::PointXYZ,pcl::PointXYZ> mls; // 定义最小二乘实现的对象mls
//     mls.setComputeNormals(false);  //设置在最小二乘计算中是否需要存储计算的法线
//     mls.setInputCloud(cloud_filtered);         //设置待处理点云
//     mls.setPolynomialOrder(2);            // 拟合2阶多项式拟合
//     mls.setPolynomialFit(false);     // 设置为false可以 加速 smooth
//     mls.setSearchMethod(treeSampling);         // 设置KD-Tree作为搜索方法
//     mls.setSearchRadius(0.05);           // 单位m.设置用于拟合的K近邻半径
//     mls.process(mls_point); 
    
//     cloud_smoothed = mls_point.makeShared();
    
//     // 法线估计
//     pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> normalEstimation;             //创建法线估计的对象
//     normalEstimation.setInputCloud(cloud_smoothed);                         //输入点云
//     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);// 创建用于最近邻搜索的KD-Tree
//     normalEstimation.setSearchMethod(tree);
//     pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); // 定义输出的点云法线

//     // K近邻确定方法，使用k个最近点，或者确定一个以r为半径的圆内的点集来确定都可以，两者选1即可
//     normalEstimation.setKSearch(10);// 使用当前点周围最近的10个点
//     //normalEstimation.setRadiusSearch(0.03);            //对于每一个点都用半径为3cm的近邻搜索方式
//     normalEstimation.compute(*normals);               //计算法线
//     // 输出法线
//     std::cout<<"normals: "<<normals->size()<<", "<<"normals fields: "<<pcl::getFieldsList(*normals)<<std::endl;
//     // pcl::io::savePCDFileASCII("/home/xiaohu/learn_SLAM/zuoye15/作业15-点云平滑及法线估计及显示/data/normals.pcd",*normals);
	
// 	// 将点云位姿、颜色、法线信息连接到一起
//     pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
//     pcl::concatenateFields(*cloud_smoothed, *normals, *cloud_with_normals);
//     // pcl::io::savePCDFileASCII("/home/xiaohu/learn_SLAM/zuoye15/作业15-点云平滑及法线估计及显示/data/cloud_with_normals.pcd",*cloud_with_normals);
	
// 	// 贪心投影三角化
//     //定义搜索树对象
//     pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
//     tree2->setInputCloud(cloud_with_normals);

//     // 三角化
//     pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;   // 定义三角化对象
//     pcl::PolygonMesh triangles; //存储最终三角化的网络模型

//     // 设置三角化参数
//     gp3.setSearchRadius(0.1);  //设置搜索时的半径，也就是KNN的球半径
//     gp3.setMu (2.5);  //设置样本点搜索其近邻点的最远距离为2.5倍（典型值2.5-3），这样使得算法自适应点云密度的变化
//     gp3.setMaximumNearestNeighbors (100);    //设置样本点最多可搜索的邻域个数，典型值是50-100

//     gp3.setMinimumAngle(M_PI/18); // 设置三角化后得到的三角形内角的最小的角度为10°
//     gp3.setMaximumAngle(2*M_PI/3); // 设置三角化后得到的三角形内角的最大角度为120°

//     gp3.setMaximumSurfaceAngle(M_PI/4); // 设置某点法线方向偏离样本点法线的最大角度45°，如果超过，连接时不考虑该点
//     gp3.setNormalConsistency(false);  //设置该参数为true保证法线朝向一致，设置为false的话不会进行法线一致性检查

//     gp3.setInputCloud (cloud_with_normals);     //设置输入点云为有向点云
//     gp3.setSearchMethod (tree2);   //设置搜索方式
//     gp3.reconstruct (triangles);  //重建提取三角化

//     // 显示网格化结果
//     // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//     // viewer->setBackgroundColor(0, 0, 0);  //
//     // viewer->addPolygonMesh(triangles, "wangge");  //
//     return triangles;
// }


// void preProcess(pcl::PointCloud<pcl::PointXYZ>::Ptr &pcloud) {
//     //体素化下采样******************************************************
// 	pcl::VoxelGrid<pcl::PointXYZ> vox;
// 	pcl::PointCloud<pcl::PointXYZ>::Ptr vox_cloud(new pcl::PointCloud<pcl::PointXYZ>);
// 	vox.setInputCloud(pcloud);
// 	vox.setLeafSize(0.1, 0.1, 0.1);
// 	vox.filter(*vox_cloud);
// 	//去除噪声点********************************************************
// 	pcl::StatisticalOutlierRemoval<pcl::PointXYZ>sor;
// 	pcl::PointCloud<pcl::PointXYZ>::Ptr sor_cloud(new pcl::PointCloud<pcl::PointXYZ>);
// 	sor.setMeanK(10);
// 	sor.setInputCloud(vox_cloud);
// 	sor.setStddevMulThresh(0.2);
// 	sor.filter(*sor_cloud);
//     pcloud = sor_cloud->makeShared();
// }

// std::vector<pcl::PointIndices> euclideanCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr pcloud, double thres) {

//     std::vector<pcl::PointIndices>ece_inlier;
// 	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
// 	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
// 	ece.setInputCloud(pcloud);
// 	ece.setClusterTolerance(thres);
// 	ece.setMinClusterSize(1);
// 	ece.setMaxClusterSize(20000);
// 	ece.setSearchMethod(tree);
// 	ece.extract(ece_inlier);
//     return ece_inlier;
// }

// void featureExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud) {
// 	preProcess(pointcloud);
// 	pcl::PointCloud<PoinT>::Ptr sor_cloud = pointcloud->makeShared();
// 	//平面分割(RANSAC)********************************************************
// 	pcl::SACSegmentation<PoinT> sac;
// 	pcl::PointIndices::Ptr inliner(new pcl::PointIndices);
// 	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
// 	pcl::PointCloud<PoinT>::Ptr sac_cloud(new pcl::PointCloud<PoinT>);
// 	sac.setInputCloud(sor_cloud);
// 	sac.setMethodType(pcl::SAC_RANSAC);
// 	sac.setModelType(pcl::SACMODEL_PLANE);
// 	sac.setMaxIterations(100);
// 	sac.setDistanceThreshold(0.02);
// 	//提取平面(展示并输出)******************************************************
// 	pcl::PointCloud<PoinT>::Ptr ext_cloud(new pcl::PointCloud<PoinT>);
// 	pcl::PointCloud<PoinT>::Ptr ext_cloud_rest(new pcl::PointCloud<PoinT>);
// 	pcl::visualization::PCLVisualizer::Ptr viewer1(new pcl::visualization::PCLVisualizer("3d view"));

// 	int i = sor_cloud->size(), j = 0;
// 	pcl::ExtractIndices<PoinT>ext;
// 	srand((unsigned)time(NULL));//刷新时间的种子节点需要放在循环体外面
// 	while (sor_cloud->size()>i*0.3)//当提取的点数小于总数的3/10时，跳出循环
// 	{
// 		ext.setInputCloud(sor_cloud);
// 		sac.segment(*inliner, *coefficients);
// 		if (inliner->indices.size()==0)
// 		{
// 			break;
// 		}
// 		//按照索引提取点云*************
// 		ext.setIndices(inliner);
// 		ext.setNegative(false);
// 		ext.filter(*ext_cloud);
// 		ext.setNegative(true);
// 		ext.filter(*ext_cloud_rest);
// 		//*****************************
// 		*sor_cloud = *ext_cloud_rest;
// 		// stringstream ss;
// 		// ss <<"C:\\Users\\Administrator\\Desktop\\"<<"ext_plane_clouds" << j << ".pcd";//路径加文件名和后缀
// 		// io::savePCDFileASCII(ss.str(), *ext_cloud);//提取的平面点云写出
// 		int *rgb = rand_rgb();//随机生成0-255的颜色值
// 		pcl::visualization::PointCloudColorHandlerCustom<PoinT>rgb1(ext_cloud,rgb[0],rgb[1],rgb[2]);//提取的平面不同彩色展示
// 		delete[]rgb;
// 		viewer1->addPointCloud(ext_cloud, rgb1,std::to_string(j));
// 		j++;
// 	}
// 	viewer1->spinOnce(1000);
// 	//欧式聚类*******************************************************
// 	std::vector<pcl::PointIndices>ece_inlier;
// 	pcl::search::KdTree<PoinT>::Ptr tree(new pcl::search::KdTree<PoinT>);
// 	pcl::EuclideanClusterExtraction<PoinT> ece;
// 	ece.setInputCloud(sor_cloud);
// 	ece.setClusterTolerance(0.02);
// 	ece.setMinClusterSize(100);
// 	ece.setMaxClusterSize(20000);
// 	ece.setSearchMethod(tree);
// 	ece.extract(ece_inlier);
// 	//聚类结果展示***************************************************
// 	pcl::visualization::PCLVisualizer::Ptr viewer2(new pcl::visualization::PCLVisualizer("Result of EuclideanCluster"));
// 	ext.setInputCloud(sor_cloud);
// 	srand((unsigned)time(NULL));
// 	for (int i = 0; i < ece_inlier.size();i++)
// 	{
// 		pcl::PointCloud<PoinT>::Ptr cloud_copy(new pcl::PointCloud<PoinT>);
// 		std::vector<int> ece_inlier_ext = ece_inlier[i].indices;
// 		pcl::copyPointCloud(*sor_cloud, ece_inlier_ext, *cloud_copy);//按照索引提取点云数据
// 		int *rgb1 = rand_rgb();
// 		pcl::visualization::PointCloudColorHandlerCustom<PoinT>rgb2(ext_cloud, rgb1[0], rgb1[1], rgb1[2]);
// 		delete[]rgb1;
// 		viewer2->addPointCloud(cloud_copy, rgb2,std::to_string(j));
// 		j++;
// 	}
// 	viewer2->spin();
// }

// // CubeResult getCube(
// //     pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud) {
// //     pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
// //     feature_extractor.setInputCloud(pointcloud);
// //     feature_extractor.compute();
 
// //     std::vector <float> moment_of_inertia;
// //     std::vector <float> eccentricity;
// //     pcl::PointXYZ min_point_AABB;
// //     pcl::PointXYZ max_point_AABB;
// //     pcl::PointXYZ min_point_OBB;
// //     pcl::PointXYZ max_point_OBB;
// //     pcl::PointXYZ position_OBB;
// //     Eigen::Matrix3f rotational_matrix_OBB;
// //     float major_value, middle_value, minor_value;
// //     Eigen::Vector3f major_vector, middle_vector, minor_vector;
// //     Eigen::Vector3f mass_center;
 
// //     feature_extractor.getMomentOfInertia(moment_of_inertia);
// //     feature_extractor.getEccentricity(eccentricity);
// //     feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
// //     feature_extractor.getEigenValues(major_value, middle_value, minor_value);
// //     feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
// //     feature_extractor.getMassCenter(mass_center);
    
// //     double dx = max_point_OBB.x - min_point_OBB.x;
// //     double dy = max_point_OBB.y - min_point_OBB.y;
// //     double dz = max_point_OBB.z - min_point_OBB.z;
// //     Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
// //     Eigen::Quaternionf quat(rotational_matrix_OBB);
// //     return CubeResult(position, quat, rotational_matrix_OBB, dx, dy, dz);
// //     //obb外接立方体，最小外接立方体
// //     // viewer->addCube(position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");
// //     // viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");
 
// //     //中心点处加坐标
// //     // pcl::PointXYZ center(mass_center(0), mass_center(1), mass_center(2));
// //     // pcl::PointXYZ x_axis(major_vector(0) + mass_center(0), major_vector(1) + mass_center(1), major_vector(2) + mass_center(2));
// //     // pcl::PointXYZ y_axis(middle_vector(0) + mass_center(0), middle_vector(1) + mass_center(1), middle_vector(2) + mass_center(2));
// //     // pcl::PointXYZ z_axis(minor_vector(0) + mass_center(0), minor_vector(1) + mass_center(1), minor_vector(2) + mass_center(2));
// //     // viewer->addLine(center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
// //     // viewer->addLine(center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
// //     // viewer->addLine(center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");
 
// //     // while (!viewer->wasStopped())
// //     // {
// //     //     viewer->spinOnce(100);
// //     //     std::this_thread::sleep_for(100ms);
// //     // }
// // }
// // void filterBYCondition(pcl::PointCloud<pcl::PointXYZ>::Ptr &pcloud) {
    
// //     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
// //     pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());   //创建条件定义对象
    
//     // range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
//     //   pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.0)));   //添加在Z字段上大于0的比较算子
// //     range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
// //       pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 0.8)));   //添加在Z字段上小于0.8的比较算子
    
// //     pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
// //     condrem.setCondition (range_cond);               
// //     condrem.setInputCloud (pcloud);                   //输入点云
// //     condrem.setKeepOrganized(true);               //设置保持点云的结构
// //     condrem.filter (*cloud_filtered);

// //     pcloud = cloud_filtered->makeShared();
// // }
// inline bool filterFuncMapClass::filtPX(double x, double y, double z, double thres) {
//     return x < thres;
// }

// inline bool filterFuncMapClass::filtPY(double x, double y, double z, double thres) {
//     return y < thres;
// }

// inline bool filterFuncMapClass::filtPZ(double x, double y, double z, double thres) {
//     return z < thres;
// }

// inline bool filterFuncMapClass::filtPR(double x, double y, double z, double thres) {
//     return std::sqrt(x * x + y * y + z * z) < thres;
// }

// inline bool filterFuncMapClass::filtOPX(double x, double y, double z, double thres) {
//     return - x < thres;
// }

// inline bool filterFuncMapClass::filtOPY(double x, double y, double z, double thres) {
//     return - y < thres;
// }

// inline bool filterFuncMapClass::filtOPZ(double x, double y, double z, double thres) {
//     return - z < thres;
// }

// inline bool filterFuncMapClass::filtOPR(double x, double y, double z, double thres) {
//     return - std::sqrt(x * x + y * y + z * z) < thres;
// }

// std::map<PCL_COMPARE, FunctionPtr> filterFuncMap = {
//     std::pair<PCL_COMPARE, FunctionPtr>(PCL_COMPARE::PX, &filterFuncMapClass::filtPX),
//     std::pair<PCL_COMPARE, FunctionPtr>(PCL_COMPARE::PY, &filterFuncMapClass::filtPY),
//     std::pair<PCL_COMPARE, FunctionPtr>(PCL_COMPARE::PZ, &filterFuncMapClass::filtPZ),
//     std::pair<PCL_COMPARE, FunctionPtr>(PCL_COMPARE::PR, &filterFuncMapClass::filtPR),
//     std::pair<PCL_COMPARE, FunctionPtr>(PCL_COMPARE::OPX, &filterFuncMapClass::filtOPX),
//     std::pair<PCL_COMPARE, FunctionPtr>(PCL_COMPARE::OPY, &filterFuncMapClass::filtOPY),
//     std::pair<PCL_COMPARE, FunctionPtr>(PCL_COMPARE::OPZ, &filterFuncMapClass::filtOPZ),
//     std::pair<PCL_COMPARE, FunctionPtr>(PCL_COMPARE::OPR, &filterFuncMapClass::filtOPR)
// };
//     //                    极值     大于
// //只要大于极值的部分
// void filterBYCondition(
//     pcl::PointCloud<pcl::PointXYZ>::Ptr &pcloud,
//     const PclFilter &thres) {

//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//     double dis = 0, x, y, z;
//     for(int i = 0; i < pcloud->points.size(); ++i) {
//         x = pcloud->points[i].x;
//         y = pcloud->points[i].y;
//         z = pcloud->points[i].z;
//         bool isinlier = true;
//         for (size_t j = 0; j < thres.size(); ++j) {
//             isinlier = filterFuncMap[thres[j].second](x, y, z, thres[j].first);
//             // switch (thres[j].second) {
//             // case PCL_COMPARE::PX:
//             //     isinlier = (x < thres[j].first);
//             //     break;
//             // case PCL_COMPARE::PY:
//             //     isinlier = (y < thres[j].first);
//             //     break;
//             // case PCL_COMPARE::PZ:
//             //     isinlier = (z < thres[j].first);
//             //     break;
//             // case PCL_COMPARE::PR:
//             //     dis = std::sqrt(x * x + y * y + z * z);
//             //     isinlier = (dis < thres[j].first);
//             //     break;
//             //  case PCL_COMPARE::OPR:
//             //     dis = std::sqrt(x * x + y * y + z * z);
//             //     isinlier = (-dis < thres[j].first);
//             //     break;
//             // case PCL_COMPARE::OPX:
//             //     isinlier = (-x < thres[j].first);
//             //     break;
//             // case PCL_COMPARE::OPY:
//             //     isinlier = (-y < thres[j].first);
//             //     break;
//             // case PCL_COMPARE::OPZ:
//             //     isinlier = (-z < thres[j].first);
//             //     break;
//             // default:
//             //     std::cout << "error type!\n";
//             //     break;
//             // }
//             if (!isinlier) break;
//         }
//         if (!isinlier) continue;
//         cloud_filtered->points.push_back(pcloud->points[i]);
//     }

//     pcloud = cloud_filtered->makeShared();
// }

// void GridPCloud::constructGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr &pcloud, double resolution) {
//     this->clear();
//     this->points = pcloud->makeShared();
//     int pt_size = pcloud->points.size();
//     for (int i = 0; i < pt_size; ++i) {
//         int x_id = std::round(pcloud->points[i].x * resolution);
//         int y_id = std::round(pcloud->points[i].y * resolution);
//         int z_id = std::round(pcloud->points[i].z * resolution);
//         grid[x_id][y_id][z_id].first.push_back(i);
//         grid[x_id][y_id][z_id].second.push_back(pcloud->points[i]);
//     }
// }


// void GridPCloud::update() {
//     points->points.clear();
//     for(auto xit = grid.begin(); xit != grid.end(); ++xit) {
//         int xid = xit->first;
//         for (auto yit = grid[xid].begin(); yit != grid[xid].end(); ++yit) {
//             int yid = yit->first;
//             for (auto zit = grid[xid][yid].begin(); zit != grid[xid][yid].end(); ++zit) {
//                 const std::vector<pcl::PointXYZ> &pc_vec = grid[xid][yid][zit->first].second;
//                 points->points.insert(points->points.end(), pc_vec.begin(), pc_vec.end());
//             }
//         }
//     }
// }

// pcl::PointCloud<pcl::PointXYZ>::Ptr gridCloud2PCLCloud(GridPCloud grid_cloud);pcl::PointCloud<pcl::PointXYZ>::Ptr gridCloud2PCLCloud(GridPCloud grid_cloud) {
//     // pcl::PointCloud<pcl::PointXYZ>::Ptr pcloud(new pcl::PointCloud<pcl::PointXYZ>);
//     // for(auto xit = grid_cloud.grid.begin(); xit != grid_cloud.grid.end(); ++xit) {
//     //     int xid = xit->first;
//     //     for (auto yit = grid_cloud.grid[xid].begin(); yit != grid_cloud.grid[xid].end(); ++yit) {
//     //         int yid = yit->first;
//     //         for (auto zit = grid_cloud.grid[xid][yid].begin(); zit != grid_cloud.grid[xid][yid].end(); ++zit) {
//     //             const std::vector<pcl::PointXYZ> &pc_vec = grid_cloud.grid[xid][yid][zit->first].second;
//     //             pcloud->points.insert(pcloud->points.end(), pc_vec.begin(), pc_vec.end());
//     //         }
//     //     }
//     // }
//     // return pcloud->makeShared();
//     return grid_cloud.points->makeShared();
// }

// sensor_msgs::PointCloud2 toColorPCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &pcloud, cv::Scalar color) {
//     sensor_msgs::PointCloud2 cloud_msg;
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr classified_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//     classified_cloud->points.clear();
//     for (int i = 0; i < pcloud->points.size(); ++i) {
//         pcl::PointXYZRGB point;
//         point.x = pcloud->points[i].x;
//         point.y = pcloud->points[i].y;
//         point.z = pcloud->points[i].z;
//         point.b = color[0];
//         point.g = color[1];
//         point.r = color[2];
//         classified_cloud->points.push_back(point);
//     }
//     // pcl::toROSMsg(*classified_cloud, cloud_msg);
//     return cloud_msg;
// }

}
}
