#ifndef Dbscan
#define Dbscan

#include"Point3.h"
#include"PointDataSource.h"
#include<thread>

#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl\kdtree\kdtree_flann.h>

namespace dbscan {
	const int NOISE = -2;	//噪声类别
	const int NOT_CLASSIFIED = -1;	//未分类

	template<typename T>
	class  DBSCAN
	{
	public:

		DBSCAN() = default;

		DBSCAN(float Neighbourhood, int MinPts, std::vector<Point3<T>>& pointCloud)
			:Neighbourhood(Neighbourhood), MinPts(MinPts), pointCloud(pointCloud) {

		}

		~DBSCAN() {

		}

		std::vector<std::vector<size_t>> GetClusterPointSet() {
			std::vector<std::vector<size_t>> cluster;
			std::vector<size_t> kernelObj;
			neighbourPoints.resize(pointCloud.size());
			neighbourDistance.resize(pointCloud.size());

			//遍历点云中所有的点找出所有的核心对象及相邻点
			/*for (size_t i = 0; i < pointCloud.size(); i++)
			{
				for (size_t j = 0; j < pointCloud.size(); j++)
				{
					if (i == j) continue;
					Point3<T> point = pointCloud[i];
					if (point.getSqaureDistanceTo(pointCloud[j])<=Neighbourhood)
					{
						neighbourPoints[i].push_back(j);
					}
				}
			}*/
			SelectKernelAndNeighbour(kernelObj);

			//迭代标记同一聚类点
			int k = -1;	//初始化聚类簇数
			//for (size_t i = 0; i < pointCloud.size(); i++)
			//{
			//	if (pointCloud[i].cluster != NOT_CLASSIFIED) continue;		//判断点有没有分类

			//	if (neighbourPoints[i].size() >= MinPts)
			//	{
			//		IterativeMarkCluster(i,++k);
			//	}
			//	else
			//	{
			//		pointCloud[i].cluster = NOISE;
			//	}
			//}
			std::cout << "根据核心对象进行聚类..." << std::endl;
			for (int i = 0; i < kernelObj.size(); i++)
			{
				if (pointCloud[kernelObj[i]].cluster!= NOT_CLASSIFIED)
				{
					continue;
				}
				std::vector<T> queue;
				queue.push_back(kernelObj[i]);
				pointCloud[kernelObj[i]].cluster = ++k;
				while (!queue.empty()) 
				{
					size_t index = queue.back();	//弹出最后一个核心对象
					queue.pop_back();

					if (neighbourPoints[index].size()>MinPts)
					{
						for (int j = 0; j < neighbourPoints[index].size(); j++)
						{
							if (k==pointCloud[neighbourPoints[index][j]].cluster)
							{
								continue;
							}
							queue.push_back(neighbourPoints[index][j]);
							pointCloud[neighbourPoints[index][j]].cluster = k;
						}
					}
				}
			}
			std::cout << "聚类结束" << std::endl;

			cluster.resize(k+1);
			for (size_t i = 0; i < pointCloud.size(); i++)
			{
				if (pointCloud[i].cluster!=NOISE)
				{
					cluster[pointCloud[i].cluster].push_back(i);
				}
			}

			return cluster;
		}

	private:
		float Neighbourhood;
		int MinPts;
		PointDataSource<T> pointCloud;
		std::vector<std::vector<int>> neighbourPoints;
		std::vector<std::vector<float>> neighbourDistance;		//pcl中的默认距离是float

		/*void IterativeMarkCluster(size_t pointIndex,int clusterClass) {
			pointCloud[pointIndex].cluster = clusterClass;
			if (neighbourPoints[pointIndex].size()<MinPts)
			{
				return;
			}

			for (auto& index: neighbourPoints[pointIndex])
			{
				if (pointCloud[index].cluster != NOT_CLASSIFIED) continue;
				IterativeMarkCluster(index, clusterClass);
			}
		}*/

		//建立核心对象
		void SelectKernelAndNeighbour(std::vector<size_t>& kernelObj) {
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
			cloud->points.resize(pointCloud.size());
			for (size_t i = 0; i < pointCloud.size(); i++)
			{
				cloud->points[i].x = pointCloud[i].x;
				cloud->points[i].y = pointCloud[i].y;
				cloud->points[i].z = pointCloud[i].z;
			}

			pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
			kdtree.setInputCloud(cloud);
			//pcl::PointXYZ searchPoint;
			std::cout << "开始建立核心对象..." << std::endl;
			//单线程进行数据查询操作
			//for (size_t i = 0; i < pointCloud.size(); i++)
			//{
			//	//searchPoint.x = pointCloud[i].x;
			//	//searchPoint.y = pointCloud[i].y;
			//	//searchPoint.z = pointCloud[i].z;
			//	kdtree.radiusSearch(cloud->points[i], Neighbourhood, neighbourPoints[i], neighbourDistance[i]);
			//	if (neighbourPoints[i].size()>=MinPts)
			//	{
			//		kernelObj.push_back(i);
			//	}
			//	else
			//	{
			//		pointCloud[i].cluster = NOISE;
			//	}
			//}

			//多线程进行数据查询
			int threadCount = 8;
			size_t span = pointCloud.size() / threadCount;
			std::vector<std::thread*> threads;		//这里如果不是指针，则会发生thread对象的赋值现象，这是不被允许的
			std::vector<std::vector<size_t>> kernelObjTmps(threadCount);		//储存每个线程中所获得的核心
			for (int i = 0; i < threadCount; i++) {
				std::thread* myThread = new std::thread(&dbscan::DBSCAN<T>::ConcurrentQuery, this,cloud, span*i,
					span*(i + 1), kdtree, std::ref(kernelObjTmps[i]));
				std::cout << "线程" << myThread->get_id() << "创建成功！" << std::endl;
				threads.push_back(myThread);
			}

			for (int i = 0; i < threads.size(); i++) {			//收尸
				threads[i]->join();
				delete threads[i];
				kernelObj.insert(kernelObj.end(), kernelObjTmps[i].begin(), kernelObjTmps[i].end());
			}

			std::cout << "核心对象建立结束!" << std::endl;
		}

		//并行运算的函数
		void ConcurrentQuery(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, size_t start, size_t end,
			pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree, std::vector<size_t>& kernelObjTmp) {
			for (size_t i = start; i < end; i++)
			{
				//searchPoint.x = pointCloud[i].x;
				//searchPoint.y = pointCloud[i].y;
				//searchPoint.z = pointCloud[i].z;
				kdtree.radiusSearch(cloud->points[i], Neighbourhood, neighbourPoints[i], neighbourDistance[i]);
				if (neighbourPoints[i].size() >= MinPts)
				{
					kernelObjTmp.push_back(i);		//这里会产生访问冲突
				}
				else
				{
					pointCloud[i].cluster = NOISE;
				}
			}
			std::cout << "线程执行结束！" << std::endl;
		}

		
	};
}

#endif // !Dbscan

