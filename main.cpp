#define _CRT_SECURE_NO_WARNINGS
#define _SCL_SECURE_NO_WARNINGS
#include"DBSCAN.h"
#include<iostream>
#include<fstream>
#include<string>

using namespace dbscan;
typedef float PointType;

int main() {
	////获取点云数据
	std::string filePath = R"(H:\论文实验\数据\区域\馨月湖\原始数据\第一次分割\TXT\tree939.txt)";
	std::ifstream input(filePath);
	//获取不带路径的文件名
	std::string::size_type iPos = filePath.find_last_of('\\') + 1;
	std::string fileNameX = filePath.substr(iPos, filePath.length() - iPos);
	std::string fileName = fileNameX.substr(0, fileNameX.rfind('.'));

	std::vector<Point3<PointType>> pointCloud;
	std::string str;

	char line[500];		//创建一个缓冲区
	char place;
	std::string _x, _y, _z;
	PointType x, y, z;
	while (input.getline(line, sizeof(line)))		//按空格进行读取数据
	{
		std::stringstream words(line);
		words >> x;		//可以通过这种方式来提取出相应的字符串数值，不用使用分割函数
		words >> place;
		words >> y;
		words >> place;
		words >> z;
		pointCloud.emplace_back(x, y, z);
	}
	input.close();

	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);	//创建点云格式为pointXYZ格式的指针

	//if (-1 == pcl::io::loadPCDFile(R"(E:\论文\论文数据\点云数据\实验\三维凸包\馨月湖\las\0.2\树干\test1.pcd)", *cloud)) //打开点云文件
	//{
	//	std::cout << "error input!" << std::endl;
	//	system("pause");
	//	return -1;
	//}

	//for (size_t i = 0; i < cloud->size(); i++)
	//{
	//	pointCloud.emplace_back((*cloud)[i].x, (*cloud)[i].y, (*cloud)[i].z);
	//}

	//cloud->~PointCloud();		//手动释放内存
	std::cout << "开始聚类：" << std::endl;
	double start = clock();

	DBSCAN<PointType> dbscan(0.05f, 5, pointCloud);
	std::vector<std::vector<size_t>>cluster = dbscan.GetClusterPointSet();

	double end = clock();
	std::cout << "耗费时间为：" << (end - start)/1000 << std::endl;

	std::ofstream output;
	std::cout << "聚类数目为：" << cluster.size() << std::endl;
	for (int i = 0; i < cluster.size(); i++)
	{
		if (cluster[i].size() < 1000) {
			continue;
		}
		output.open(R"(C:\Users\23547\Desktop\Test\Paper\)" +fileName+"_cluster"+ std::to_string(i + 1) + ".txt");
		//output.precision(20);		//设置输出的精度
		output.setf(std::ios::fixed, std::ios::floatfield); //设置浮点数保留小数点后的六位
		for (size_t j = 0; j < cluster[i].size(); j++)
		{
			output << pointCloud[cluster[i][j]].x << " "
				<< pointCloud[cluster[i][j]].y << " "
				<< pointCloud[cluster[i][j]].z << " "  
				<< pointCloud[cluster[i][j]].z<<std::endl;
		}
		output.close();
	}

	std::cout << "执行成功！" << std::endl;
	system("pause");

	return 0;
}