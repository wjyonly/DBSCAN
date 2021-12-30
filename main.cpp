#define _CRT_SECURE_NO_WARNINGS
#define _SCL_SECURE_NO_WARNINGS
#include"DBSCAN.h"
#include<iostream>
#include<fstream>
#include<string>

using namespace dbscan;
typedef float PointType;

int main() {
	////��ȡ��������
	std::string filePath = R"(H:\����ʵ��\����\����\ܰ�º�\ԭʼ����\��һ�ηָ�\TXT\tree939.txt)";
	std::ifstream input(filePath);
	//��ȡ����·�����ļ���
	std::string::size_type iPos = filePath.find_last_of('\\') + 1;
	std::string fileNameX = filePath.substr(iPos, filePath.length() - iPos);
	std::string fileName = fileNameX.substr(0, fileNameX.rfind('.'));

	std::vector<Point3<PointType>> pointCloud;
	std::string str;

	char line[500];		//����һ��������
	char place;
	std::string _x, _y, _z;
	PointType x, y, z;
	while (input.getline(line, sizeof(line)))		//���ո���ж�ȡ����
	{
		std::stringstream words(line);
		words >> x;		//����ͨ�����ַ�ʽ����ȡ����Ӧ���ַ�����ֵ������ʹ�÷ָ��
		words >> place;
		words >> y;
		words >> place;
		words >> z;
		pointCloud.emplace_back(x, y, z);
	}
	input.close();

	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);	//�������Ƹ�ʽΪpointXYZ��ʽ��ָ��

	//if (-1 == pcl::io::loadPCDFile(R"(E:\����\��������\��������\ʵ��\��ά͹��\ܰ�º�\las\0.2\����\test1.pcd)", *cloud)) //�򿪵����ļ�
	//{
	//	std::cout << "error input!" << std::endl;
	//	system("pause");
	//	return -1;
	//}

	//for (size_t i = 0; i < cloud->size(); i++)
	//{
	//	pointCloud.emplace_back((*cloud)[i].x, (*cloud)[i].y, (*cloud)[i].z);
	//}

	//cloud->~PointCloud();		//�ֶ��ͷ��ڴ�
	std::cout << "��ʼ���ࣺ" << std::endl;
	double start = clock();

	DBSCAN<PointType> dbscan(0.05f, 5, pointCloud);
	std::vector<std::vector<size_t>>cluster = dbscan.GetClusterPointSet();

	double end = clock();
	std::cout << "�ķ�ʱ��Ϊ��" << (end - start)/1000 << std::endl;

	std::ofstream output;
	std::cout << "������ĿΪ��" << cluster.size() << std::endl;
	for (int i = 0; i < cluster.size(); i++)
	{
		if (cluster[i].size() < 1000) {
			continue;
		}
		output.open(R"(C:\Users\23547\Desktop\Test\Paper\)" +fileName+"_cluster"+ std::to_string(i + 1) + ".txt");
		//output.precision(20);		//��������ľ���
		output.setf(std::ios::fixed, std::ios::floatfield); //���ø���������С��������λ
		for (size_t j = 0; j < cluster[i].size(); j++)
		{
			output << pointCloud[cluster[i][j]].x << " "
				<< pointCloud[cluster[i][j]].y << " "
				<< pointCloud[cluster[i][j]].z << " "  
				<< pointCloud[cluster[i][j]].z<<std::endl;
		}
		output.close();
	}

	std::cout << "ִ�гɹ���" << std::endl;
	system("pause");

	return 0;
}