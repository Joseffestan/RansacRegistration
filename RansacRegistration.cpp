// RansacRegistration.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <array>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <vector>

using namespace std;
using namespace pcl;
//计算两点距离的平方
double PointDistance(PointXYZ P1, PointXYZ P2)
{
	double ans = sqrt((P1.x - P2.x)*(P1.x - P2.x) + (P1.y - P2.y)*(P1.y - P2.y) + (P1.z - P2.z)*(P1.z - P2.z));
	return ans;
}
//计算三点是否够远
bool ThreePointsDistanceCheck(PointXYZ P1, PointXYZ P2, PointXYZ P3, int nMinDis)
{
	if (PointDistance(P1, P2) < nMinDis)
		return false;
	if (PointDistance(P1, P3) < nMinDis)
		return false;
	if (PointDistance(P3, P2) < nMinDis)
		return false;
	return true;
}
//点云可视化
void visualize_pcd(PointCloud<PointXYZ>::Ptr pcd_src, PointCloud<PointXYZ>::Ptr pcd_tgt, PointCloud<PointXYZ>::Ptr pcd_final)
{
	//int vp_1, vp_2;
	// Create a PCLVisualizer object
	pcl::visualization::PCLVisualizer viewer("registration Viewer");
	//viewer.createViewPort (0.0, 0, 0.5, 1.0, vp_1);
	// viewer.createViewPort (0.5, 0, 1.0, 1.0, vp_2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(pcd_src, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(pcd_tgt, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_h(pcd_final, 0, 0, 255);
	viewer.addPointCloud(pcd_src, src_h, "source cloud");
	viewer.addPointCloud(pcd_tgt, tgt_h, "tgt cloud");
	viewer.addPointCloud(pcd_final, final_h, "final cloud");
	//viewer.addCoordinateSystem(1.0);
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

}
int main(int argc, char** argv)
{
	PointCloud<PointXYZ>::Ptr cloud1(new PointCloud<PointXYZ>);
	io::loadPCDFile("e:\\FeatureCloud_sofa.pcd", *cloud1);
	visualize_pcd(cloud1,cloud1,cloud1);

	//读取点云
	char* strTgtFile = new char[80];
	char* strSrcFile = new char[80];
	strcpy(strTgtFile, "e:\\data\\office2.pcd");
	strcpy(strSrcFile, "e:\\data\\office1.pcd");
	PointCloud<PointXYZ>::Ptr CloudTgt(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr CloudSrc(new PointCloud<PointXYZ>);
	io::loadPCDFile(strTgtFile, *CloudTgt);
	io::loadPCDFile(strSrcFile, *CloudSrc);
	int nTgtPoint = CloudTgt->width;//查询点云文件内点的数量
	int nSrcPoint = CloudSrc->width;
	int CorrsSrc[3] = { 0 }, CorrsTgt[3] = { 0 };//对应点对索引
	double MinFitnessScore = 1000000.0;
	double MaxFitnessScore = 1.0;
	registration::TransformationEstimationSVD<PointXYZ, PointXYZ>::Matrix4 FinalTransformation;
	PointCloud<PointXYZ>::Ptr CloudSrcOutput(new PointCloud<PointXYZ>());
	int nIter = 0, nTry = 0;
	srand((unsigned int)time(0));
	for (nTry = 0; nIter < 2000; nTry++)
	{
		//cout << "---------------------------try:" << nTry << endl;
		//源点云随机选取三个符合距离要求的点s1,s2,s3
		int nRand;
		do {
			for (int i = 0; i < 3; i++)
			{
				nRand = rand();
				int p = nRand % (nSrcPoint)+1;
				CorrsSrc[i] = p;
			}
		} while (!ThreePointsDistanceCheck(CloudSrc->points[CorrsSrc[0]], CloudSrc->points[CorrsSrc[1]], CloudSrc->points[CorrsSrc[2]], 1000));
		//cout << "choose from source:\nNo." << CorrsSrc[0] << ":(" << CloudSrc->points[CorrsSrc[0]].x << "," << CloudSrc->points[CorrsSrc[0]].y << "," << CloudSrc->points[CorrsSrc[0]].z << ").\n"
		//	<< "No." << CorrsSrc[1] << ":(" << CloudSrc->points[CorrsSrc[1]].x << "," << CloudSrc->points[CorrsSrc[1]].y << "," << CloudSrc->points[CorrsSrc[1]].z << ").\n"
		//	<< "No." << CorrsSrc[2] << ":(" << CloudSrc->points[CorrsSrc[2]].x << "," << CloudSrc->points[CorrsSrc[2]].y << "," << CloudSrc->points[CorrsSrc[2]].z << ").\n";
		double dDisSrcP0P1 = PointDistance(CloudSrc->points[CorrsSrc[0]], CloudSrc->points[CorrsSrc[1]]);
		double dDisSrcP1P2 = PointDistance(CloudSrc->points[CorrsSrc[1]], CloudSrc->points[CorrsSrc[2]]);
		double dDisSrcP0P2 = PointDistance(CloudSrc->points[CorrsSrc[0]], CloudSrc->points[CorrsSrc[2]]);
		double dDisTgtP0P1 = 0, dDisTgtP1P2 = 0, dDisTgtP0P2 = 0;
		//在目标点云内随机选取一点t1
		{
			nRand = rand();
			int p = nRand % (nTgtPoint)+1;
			CorrsTgt[0] = p;
		}
		//cout << "choose from target:\nNo." << CorrsTgt[0] << ":(" << CloudTgt->points[CorrsTgt[0]].x << "," << CloudTgt->points[CorrsTgt[0]].y << "," << CloudTgt->points[CorrsTgt[0]].z << ").\n";
		//在目标点云随机寻找第二点
		int i = 0;
		vector<int> nTgtP1;
		for (i = 0; i < nTgtPoint; i++)
		{
			dDisTgtP0P1 = PointDistance(CloudTgt->points[i], CloudTgt->points[CorrsTgt[0]]);
			if (fabs(dDisTgtP0P1 - dDisSrcP0P1) < 1)
				nTgtP1.push_back(i);
		}
		if (nTgtP1.size() > 0)
		{
			nRand = rand();
			int p = nRand % (nTgtP1.size());
			CorrsTgt[1] = nTgtP1.at(p);
		}
		nTgtP1.clear();
		//cout << "No." << CorrsTgt[1] << ":(" << CloudTgt->points[CorrsTgt[1]].x << ", " << CloudTgt->points[CorrsTgt[1]].y << ", " << CloudTgt->points[CorrsTgt[1]].z << ").\n";	
		//在目标点云寻找第三点
		i = 0;
		vector<int> nTgtP2;
		for (i = 0; i < nTgtPoint; i++)
		{
			dDisTgtP0P2 = PointDistance(CloudTgt->points[CorrsTgt[0]], CloudTgt->points[i]);
			dDisTgtP1P2 = PointDistance(CloudTgt->points[CorrsTgt[1]], CloudTgt->points[i]);
			if ((fabs(dDisTgtP0P2 - dDisSrcP0P2) < 1) && (fabs(dDisTgtP1P2 - dDisSrcP1P2) < 1))
				nTgtP2.push_back(i);
		}
		if (nTgtP2.size() > 0)
		{
			nRand = rand();
			int p = nRand % (nTgtP2.size());
			CorrsTgt[2] = nTgtP2.at(p);
		}
		nTgtP2.clear();
		//cout << "No." << CorrsTgt[2] << ":(" << CloudTgt->points[CorrsTgt[2]].x << ", " << CloudTgt->points[CorrsTgt[2]].y << ", " << CloudTgt->points[CorrsTgt[2]].z << ").\n";
		if (CorrsTgt[1] == nTgtPoint - 1 || CorrsTgt[2] == nTgtPoint - 1)//目标点云中搜寻到最后一点
			continue;
		if (PointDistance(CloudTgt->points[CorrsTgt[0]], CloudSrc->points[CorrsSrc[0]]) > 500 || PointDistance(CloudTgt->points[CorrsTgt[1]], CloudSrc->points[CorrsSrc[1]]) > 500 || PointDistance(CloudTgt->points[CorrsTgt[2]], CloudSrc->points[CorrsSrc[2]]) > 500)
			continue;//对应点距离过大
		if (CorrsTgt[0] == CorrsTgt[1] || CorrsTgt[0] == CorrsTgt[2] || CorrsTgt[1] == CorrsTgt[2])
			continue;
		cout << "-----------------------------iteration:" << nIter++ << endl;
		//计算旋转矩阵
		PointCloud<PointXYZ>::Ptr CloudPointSrc(new PointCloud<PointXYZ>());
		PointCloud<PointXYZ>::Ptr CloudPointTgt(new PointCloud<PointXYZ>());
		CloudPointSrc->width = 3;
		CloudPointSrc->height = 1;
		CloudPointSrc->is_dense = false;
		CloudPointSrc->resize(CloudPointSrc->width * CloudPointSrc->height);
		CloudPointTgt->width = 3;
		CloudPointTgt->height = 1;
		CloudPointTgt->is_dense = false;
		CloudPointTgt->resize(CloudPointTgt->width * CloudPointTgt->height);
		//同名点写入点云
		CloudPointSrc->points[0] = CloudSrc->points[CorrsSrc[0]];
		CloudPointSrc->points[1] = CloudSrc->points[CorrsSrc[1]];
		CloudPointSrc->points[2] = CloudSrc->points[CorrsSrc[2]];
		CloudPointTgt->points[0] = CloudTgt->points[CorrsTgt[0]];
		CloudPointTgt->points[1] = CloudTgt->points[CorrsTgt[1]];
		CloudPointTgt->points[2] = CloudTgt->points[CorrsTgt[2]];
		//利用SVD方法求解变换矩阵
		registration::TransformationEstimationSVD<PointXYZ, PointXYZ> TESVD;
		registration::TransformationEstimationSVD<PointXYZ, PointXYZ>::Matrix4 transformation;
		TESVD.estimateRigidTransformation(*CloudPointSrc, *CloudPointTgt, transformation);
		cout << "estimated rotation:" << endl;
		printf("    | %6.3f %6.3f %6.3f | \n", transformation(0, 0), transformation(0, 1), transformation(0, 2));
		printf("R = | %6.3f %6.3f %6.3f | \n", transformation(1, 0), transformation(1, 1), transformation(1, 2));
		printf("    | %6.3f %6.3f %6.3f | \n", transformation(2, 0), transformation(2, 1), transformation(2, 2));
		printf("\n");
		printf("t = < %0.3f, %0.3f, %0.3f >\n", transformation(0, 3), transformation(1, 3), transformation(2, 3));
		//旋转源点云,计算匹配分数
		double FitnessScore = 0.0;
		transformPointCloud(*CloudSrc, *CloudSrcOutput, transformation);
		KdTreeFLANN<PointXYZ> kdtree;
		kdtree.setInputCloud(CloudTgt);
		int K = 10;
		std::vector<int> nn_indices(K);
		std::vector<float> nn_dists(K);
		int nr = 0;
		for (i = 0; i < CloudSrcOutput->points.size(); i = i + 4)
		{
			// Find its nearest neighbor in the target
			kdtree.nearestKSearch(CloudSrcOutput->points[i], 1, nn_indices, nn_dists);
			// Deal with occlusions (incomplete targets)//部分被遮挡
			if (nn_dists[0] <= 5000)
			{
				// Add to the fitness score
				FitnessScore += nn_dists[0];
				nr++;
			}
		}
		if (nr > 0)
			FitnessScore = (FitnessScore / nr);
		else
			FitnessScore = std::numeric_limits<double>::max();
		cout << "FitnessScore is:" << FitnessScore << endl;
		if (FitnessScore < MinFitnessScore)
		{
			MinFitnessScore = FitnessScore;
			FinalTransformation = transformation;
		}
	}
	//输出最终旋转矩阵
	transformPointCloud(*CloudSrc, *CloudSrcOutput, FinalTransformation);
	FILE* pfOutputTransformation;
	char* strOutputTransformation = new char[80];
	strcpy(strOutputTransformation, "e:\\Transformation.txt");
	fopen_s(&pfOutputTransformation, strOutputTransformation, "w");
	if (pfOutputTransformation == NULL) return 1;
	fprintf(pfOutputTransformation, "Min Fitness Score is:%f\n", MinFitnessScore);
	fprintf(pfOutputTransformation, "estimated rotation of \"%s\" and \"%s\" :\n", strTgtFile, strSrcFile);
	fprintf(pfOutputTransformation, "    | %6.3f %6.3f %6.3f | \n", FinalTransformation(0, 0), FinalTransformation(0, 1), FinalTransformation(0, 2));
	fprintf(pfOutputTransformation, "R = | %6.3f %6.3f %6.3f | \n", FinalTransformation(1, 0), FinalTransformation(1, 1), FinalTransformation(1, 2));
	fprintf(pfOutputTransformation, "    | %6.3f %6.3f %6.3f | \n", FinalTransformation(2, 0), FinalTransformation(2, 1), FinalTransformation(2, 2));
	fprintf(pfOutputTransformation, "\n");
	fprintf(pfOutputTransformation, "t = < %0.3f, %0.3f, %0.3f >\n", FinalTransformation(0, 3), FinalTransformation(1, 3), FinalTransformation(2, 3));
	fclose(pfOutputTransformation);
	delete[] strOutputTransformation;
	strOutputTransformation = NULL;
	delete[] strTgtFile;
	strTgtFile = NULL;
	delete[] strSrcFile;
	strSrcFile = NULL;
	//输出旋转后源点云+目标点云
	io::savePCDFileASCII("e:\\OutputCloud.pcd", *CloudSrcOutput);
	visualize_pcd(CloudSrc, CloudTgt, CloudSrcOutput);
	return 0;
}
