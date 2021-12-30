#ifndef POINTDATASOURCE
#define POINTDATASOURCE

#include"Point3.h"
#include<vector>

namespace dbscan {

	template<typename T>
	class  PointDataSource
	{
	public:
		PointDataSource(Point3<T>* ptr,size_t count):m_ptr(ptr),m_count(count) {

		 }

		PointDataSource(std::vector<Point3<T>>& pointCloud) :m_ptr(&pointCloud[0]), m_count(pointCloud.size()) {

		}

		PointDataSource():m_ptr(nullptr),m_count(0) {

		}

		~PointDataSource(){

		}

		PointDataSource& operator=(const PointDataSource& other) = default;

		size_t size() {
			return m_count;
		}

		Point3<T>& operator[](size_t index){
			return m_ptr[index];
		}

		const Point3<T>* begin() {
			return m_ptr;
		}

		const Point3<T>* end() {
			return m_ptr + m_count;
		}

	private:
		Point3<T>* m_ptr;
		size_t m_count;

	};
}

#endif // ! POINTDATASOURCE

