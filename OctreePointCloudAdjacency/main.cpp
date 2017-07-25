#include <pcl/octree/octree.h>
#include <pcl/octree/octree_container.h>
#include <pcl/io/pcd_io.h>  


#include <pcl/segmentation/supervoxel_clustering.h>  

#include <pcl/point_types.h>
#include <pcl/point_cloud.h> 
#include <pcl/octree/impl/octree_pointcloud.hpp>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_pointcloud_adjacency_container.h>
#include <pcl/octree/octree_pointcloud_adjacency.h>

int main()
{
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->width = 1000;
	cloud->height = 1;
	cloud->points.resize(cloud->width*cloud->height);

	for (size_t i = 0; i < cloud->points.size();++i)
	{
		cloud->points[i].x = 1024.0f*rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024.0f*rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024.0f*rand() / (RAND_MAX + 1.0f);
	}*/

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud = boost::make_shared <pcl::PointCloud<pcl::PointXYZRGBA>>();
	pcl::console::print_highlight("Loading point cloud...\n");
	if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>("test.pcd", *cloud))
	{
		pcl::console::print_error("Error loading cloud file!\n");
		return (1);
	}
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr voxel_centroid_cloud_111;


	pcl::octree::OctreePointCloudAdjacency<pcl::PointXYZRGBA, pcl::octree::OctreePointCloudAdjacencyContainer<pcl::PointXYZRGBA>, pcl::octree::OctreeContainerEmpty>::Ptr adjacency_octree_123;

	
	adjacency_octree_123.reset(new pcl::octree::OctreePointCloudAdjacency<pcl::PointXYZRGBA, pcl::octree::OctreePointCloudAdjacencyContainer<pcl::PointXYZRGBA>, pcl::octree::OctreeContainerEmpty>(128.0f));
	adjacency_octree_123->setInputCloud(cloud);

	adjacency_octree_123->addPointsFromInputCloud();


	voxel_centroid_cloud_111.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);//重置所有数据
	voxel_centroid_cloud_111->resize(adjacency_octree_123->getLeafCount());//设置八叉树分辨率，

	typedef pcl::octree::OctreePointCloudAdjacencyContainer<pcl::PointXYZRGBA, pcl::SupervoxelClustering<pcl::PointXYZRGBA>::VoxelData> LeafContainerT;
	typedef std::vector <LeafContainerT*> LeafVectorT;
	//typename LeafVectorT::iterator leaf_itr = adjacency_octree_123->begin();//此处的begin只是叶子节点容器leaf_vector 的 begin
	//typename pcl::PointCloud<pcl::PointXYZRGBA>::iterator cent_cloud_itr = voxel_centroid_cloud_111->begin();
	//for (int idx = 0; leaf_itr != adjacency_octree_123->end(); ++leaf_itr, ++cent_cloud_itr, ++idx)
	//{
	//	pcl::SupervoxelClustering::VoxelData& new_voxel_data = (*leaf_itr)->getData();
	//	//std::cout << adjacency_octree_->at(5)->getData().xyz_[0] << "  " << adjacency_octree_->at(5)->getData().xyz_[1] << "  " << adjacency_octree_->at(5)->getData().xyz_[2] << "  " << endl;
	//	//Add the point to the centroid cloud
	//	new_voxel_data.getPoint(*cent_cloud_itr);
	//	//voxel_centroid_cloud_->push_back(new_voxel_data.getPoint ());
	//	new_voxel_data.idx_ = idx;

	//}


	
	std::cout << adjacency_octree_123->at(4)->getData().x << "  " << adjacency_octree_123->at(4)->getData().y << "  " << adjacency_octree_123->at(4)->getData().z << "  " << std::endl;
	std::cout << adjacency_octree_123->at(5)->getData().x << "  " << adjacency_octree_123->at(5)->getData().y << "  " << adjacency_octree_123->at(5)->getData().z << "  " << std::endl;
	std::cout << adjacency_octree_123->at(6)->getData().x << "  " << adjacency_octree_123->at(6)->getData().y << "  " << adjacency_octree_123->at(6)->getData().z << "  " << std::endl;
	std::cout << adjacency_octree_123->at(7)->getData().x << "  " << adjacency_octree_123->at(7)->getData().y << "  " << adjacency_octree_123->at(7)->getData().z << "  " << std::endl;


	//float resolution = 128.0f;
	//pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);

	//octree.setInputCloud(cloud);
	//octree.addPointsFromInputCloud();//构建octree

	return 1;












}