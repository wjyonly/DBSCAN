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
	const int NOISE = -2;	//�������
	const int NOT_CLASSIFIED = -1;	//δ����

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

			//�������������еĵ��ҳ����еĺ��Ķ������ڵ�
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

			//�������ͬһ�����
			int k = -1;	//��ʼ���������
			//for (size_t i = 0; i < pointCloud.size(); i++)
			//{
			//	if (pointCloud[i].cluster != NOT_CLASSIFIED) continue;		//�жϵ���û�з���

			//	if (neighbourPoints[i].size() >= MinPts)
			//	{
			//		IterativeMarkCluster(i,++k);
			//	}
			//	else
			//	{
			//		pointCloud[i].cluster = NOISE;
			//	}
			//}
			std::cout << "���ݺ��Ķ�����о���..." << std::endl;
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
					size_t index = queue.back();	//�������һ�����Ķ���
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
			std::cout << "�������" << std::endl;

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
		std::vector<std::vector<float>> neighbourDistance;		//pcl�е�Ĭ�Ͼ�����float

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

		//�������Ķ���
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
			std::cout << "��ʼ�������Ķ���..." << std::endl;
			//���߳̽������ݲ�ѯ����
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

			//���߳̽������ݲ�ѯ
			int threadCount = 8;
			size_t span = pointCloud.size() / threadCount;
			std::vector<std::thread*> threads;		//�����������ָ�룬��ᷢ��thread����ĸ�ֵ�������ǲ��������
			std::vector<std::vector<size_t>> kernelObjTmps(threadCount);		//����ÿ���߳�������õĺ���
			for (int i = 0; i < threadCount; i++) {
				std::thread* myThread = new std::thread(&dbscan::DBSCAN<T>::ConcurrentQuery, this,cloud, span*i,
					span*(i + 1), kdtree, std::ref(kernelObjTmps[i]));
				std::cout << "�߳�" << myThread->get_id() << "�����ɹ���" << std::endl;
				threads.push_back(myThread);
			}

			for (int i = 0; i < threads.size(); i++) {			//��ʬ
				threads[i]->join();
				delete threads[i];
				kernelObj.insert(kernelObj.end(), kernelObjTmps[i].begin(), kernelObjTmps[i].end());
			}

			std::cout << "���Ķ���������!" << std::endl;
		}

		//��������ĺ���
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
					kernelObjTmp.push_back(i);		//�����������ʳ�ͻ
				}
				else
				{
					pointCloud[i].cluster = NOISE;
				}
			}
			std::cout << "�߳�ִ�н�����" << std::endl;
		}

		
	};
}

#endif // !Dbscan

