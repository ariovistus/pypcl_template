#include <boost/python.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <Python.h>

using namespace std;
using namespace boost::python;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT>::Ptr CloudPtr;

uint8_t interpolate(float z, float z1, float z2, uint8_t b1, uint8_t b2) {
    return (b2 * (z-z1) + b1 * (z2-z)) / (z2-z1);
}

void interpolateColor(PointT &pt, float min_z, float max_z) {
#define N 3
    float zpts[N] = {min_z, (min_z + max_z) / 2, max_z};
    uint8_t rgbs[N*3] = {
        255, 15, 15,
        15, 255, 15,
        15, 15, 255
    };
    for(size_t i(1); i < N; i++) {
        float z1 = zpts[i-1];
        float z2 = zpts[i];
        if(pt.z < z2) {
            uint8_t r1 = rgbs[3*(i-1)];
            uint8_t r2 = rgbs[3*i];
            uint8_t g1 = rgbs[3*(i-1)+1];
            uint8_t g2 = rgbs[3*i+1];
            uint8_t b1 = rgbs[3*(i-1)+2];
            uint8_t b2 = rgbs[3*i+2];
            uint8_t r = interpolate(pt.z, z1, z2, r1, r2);
            uint8_t g = interpolate(pt.z, z1, z2, g1, g2);
            uint8_t b = interpolate(pt.z, z1, z2, b1, b2);

            pt.rgb = (static_cast<uint32_t>(r) << 16 |
                    static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
            break;
        }
    }
}

void loadCloud(pcl::PointCloud<PointT>::Ptr cloud, object np_xyz) {
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

    float min_z = 100000;
    float max_z = -100000;
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
            if(min_z > pt.z) {
                min_z = pt.z;
            }
            if(max_z < pt.z) {
                max_z = pt.z;
            }
            //cout << "xyz[" << r << ", " << c << "]: [";
            //cout << pt.x << ", " << pt.y << ", " << pt.z << "]" << endl;
        }
    }

    for(size_t r(0); r < cloud->height; r++) {
        for(size_t c(0); c < cloud->width; c++) {
            PointT &pt = cloud->at(c, r);
            interpolateColor(pt, min_z, max_z);
        }
    }
}

CloudPtr cloudFromNumpyXYZ(object np_xyz) {
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    loadCloud(cloud, np_xyz);
    return cloud;
}

struct Viewer {
    pcl::visualization::PCLVisualizer *viewer;
    string cloudName = "da cloud";
    CloudPtr cloud;

    Viewer() {
        viewer = new pcl::visualization::PCLVisualizer("Cloud Viewer");
        viewer->setBackgroundColor(0, 0, 0);
        viewer->addCoordinateSystem(1.0);
        viewer->initCameraParameters();
    }

    void setupCloud(CloudPtr _cloud) {
        this->cloud = _cloud;
        pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
        viewer->addPointCloud<PointT>(cloud, rgb, cloudName);
        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
            1,
            cloudName
        );
    }

    bool wasStopped() {
        return viewer->wasStopped();
    }

    void spin() {
        viewer->spinOnce(100);
    }

    void updateCloud(object np_xyz) {
        loadCloud(cloud, np_xyz);
        viewer->updatePointCloud(cloud, cloudName);
    }

    void close() {
        viewer->close();
    }
};

Viewer *process(object np_xyz) {
    CloudPtr cloud = cloudFromNumpyXYZ(np_xyz);
    Viewer *viewer = new Viewer();
    viewer->setupCloud(cloud);

    return viewer;
}

BOOST_PYTHON_MODULE(libpclproc)
{
    def("process", process, return_value_policy<manage_new_object>());
    class_<Viewer>("Viewer")
        .def("wasStopped", &Viewer::wasStopped)
        .def("updateCloud", &Viewer::updateCloud)
        .def("close", &Viewer::close)
        .def("spin", &Viewer::spin);
}
