#ifndef POINT3
#define POINT3

namespace dbscan {

	template<typename T>
	class  Point3
	{
	public:
		T x, y, z;
		int cluster= NOT_CLASSIFIED;

		Point3(T x,T y,T z):x(x),y(y),z(z){

		 }

		~Point3() {

		}

		Point3 operator-(const Point3& other) {
			return Point3(x - other.x, y - other.y, z - other.z);
		}

		Point3 operator+(const Point3& other) {
			return Point3(x + other.x, y + other.y, z + other.z);
		}

		T getSqaureDistanceTo(const Point3& other) {
			T dx = x - other.x;
			T dy = y - other.y;
			T dz = z - other.z;
			return (dx*dx + dy*dy + dz*dz);
		}
	private:

	};
}

#endif // !POINT3

