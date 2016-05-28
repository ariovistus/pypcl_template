#include <boost/python.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <Python.h>

using namespace std;
using namespace boost::python;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT>::Ptr CloudPtr;

CloudPtr cloudFromNumpyXYZ(object np_xyz) {
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    Py_buffer buffer;

    if(
            len(np_xyz.attr("shape")) != 3 || 
            extract<int>(np_xyz.attr("shape")[0]) != 3) {
        throw ("expected numpy array to have shape (3, height, width)");
    }
    cloud->height = extract<int>(np_xyz.attr("shape")[1]);
    cloud->width = extract<int>(np_xyz.attr("shape")[2]);
    cloud->resize(cloud->width * cloud->height);

    int result = PyObject_GetBuffer(np_xyz.ptr(), &buffer, PyBUF_ND | PyBUF_C_CONTIGUOUS);
    if(result != 0) {
        throw ("expected np_xyz to be buffer interface");
    }

    for(size_t r = 0; r < cloud->height; r++) {
        void *offset = buffer.buf + r * buffer.strides[1];
        for(size_t c = 0; c < cloud->width; c++, offset += buffer.strides[2]) {
            void *px = 0 * buffer.strides[0] + offset;
            void *py = 1 * buffer.strides[0] + offset;
            void *pz = 2 * buffer.strides[0] + offset;
            PointT &pt = cloud->at(c, r);
            pt.x = *((float*)px) / 1000.;
            pt.y = *((float*)py) / 1000.;
            pt.z = *((float*)pz) / 1000.;
            //cout << "xyz[" << r << ", " << c << "]: [";
            //cout << pt.x << ", " << pt.y << ", " << pt.z << "]" << endl;
        }
    }

    return cloud;
}

object process(object np_xyz) {
    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");

    CloudPtr cloud = cloudFromNumpyXYZ(np_xyz);

    viewer.showCloud(cloud);

    while(!viewer.wasStopped()) {
    }

    return np_xyz;
}

BOOST_PYTHON_MODULE(libpclproc)
{
    def("process", process);
}
